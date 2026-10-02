"""Photo tour and explicitly approved return, inside the existing mission worker."""
from copy import deepcopy
from dataclasses import replace,asdict
from pathlib import Path
import json,os,threading,uuid
from .mission_types import MissionType,MissionRunResult,SegmentExecutionStatus,SegmentContext,MissionProgress
from .configuration import Location


class PhotoMission:
    def __init__(self, settings, output, executor, mail=None):
        self.settings=settings
        self.output=Path(output)/'photos'
        self.executor=executor
        self.mail=mail
        self.mail_recipient=""
        self.mail_error=""
        self.lock=threading.RLock()
        self.session={}
        self.manual_capture=dict(state="IDLE",busy=False)
        self.capture_stop=threading.Event()
        recipient_path=self.output/'mail-recipient.json'
        if recipient_path.exists():
            try:
                from .mail_outbox import address
                value=json.loads(recipient_path.read_text()).get('recipient','')
                self.mail_recipient=address(value) if value else ''
            except (OSError,ValueError,TypeError,AttributeError):
                self.mail_error='저장된 수신 주소를 확인하세요.'
        path=self.output/'session.json'
        if path.exists():
            value=json.loads(path.read_text())
            if value.get('version')!=1:raise ValueError('unknown photo session version')
            self.session=value
            if value.get('stage') in ('OUTBOUND','RETURNING','PHOTO_ARRIVED','RETURN_ARRIVED'):
                self.session.update(stage='INTERRUPTED',detail='실행 중 재시작됨 · 자동 재개하지 않음')

    def mail_settings(self, recipient):
        from .mail_outbox import address
        if not isinstance(recipient,str):raise ValueError('메일 주소를 입력하세요.')
        recipient=address(recipient.strip()) if recipient.strip() else ''
        with self.lock:
            self.output.mkdir(parents=True,exist_ok=True)
            path=self.output/'mail-recipient.json';temp=path.with_suffix('.tmp')
            with os.fdopen(os.open(str(temp),os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600),'w') as stream:
                json.dump(dict(recipient=recipient),stream);stream.flush();os.fsync(stream.fileno())
            temp.replace(path);self.mail_recipient=recipient;self.mail_error=''
        return dict(saved=True,recipient=recipient,note='다음 촬영 임무부터 적용됩니다.' if recipient else '다음 촬영 임무는 메일을 보내지 않습니다.')

    def mail_snapshot(self):
        state=self.mail.snapshot() if self.mail is not None else dict(configured=False,jobs=[],reason='메일 작업 미설정')
        with self.lock:
            return dict(**state,recipient=self.mail_recipient,session_recipient=self.session.get('mail_recipient',''),enqueue_error=self.mail_error,manual_capture=deepcopy(self.manual_capture))

    def capture_and_mail(self, recipient):
        """Independent still capture: never changes the tour, navigation or ownership."""
        from .mail_outbox import address
        if not isinstance(recipient,str) or not recipient.strip():
            raise ValueError('사진을 받을 메일 주소를 입력하세요.')
        recipient=address(recipient.strip())
        if self.mail is None:raise ValueError('메일 작업 미설정')
        with self.lock:
            if self.manual_capture.get('busy'):raise ValueError('현재 사진 촬영을 처리 중입니다.')
            if self.capture_stop.is_set():raise ValueError('촬영 기능 종료 중')
            capture_id='manual_'+uuid.uuid4().hex
            self.manual_capture=dict(id=capture_id,state='CAPTURING',busy=True,recipient=recipient,
                                     path='',job_id='',detail='새 RGB 프레임을 기다립니다.')
            worker=threading.Thread(target=self._capture_and_mail,args=(capture_id,recipient),daemon=True)
            try:worker.start()
            except Exception:
                self.manual_capture.update(state='FAILED',busy=False,detail='촬영 작업 시작 실패')
                raise
            return dict(accepted=True,id=capture_id,recipient=recipient)

    def _capture_and_mail(self, capture_id, recipient):
        saved=''
        try:
            capture=self.executor.capture_photo('current_position',self.output/capture_id,self.capture_stop.is_set)
            if capture.status is not SegmentExecutionStatus.SUCCESS:
                raise ValueError(capture.reason or '새 RGB 사진 촬영 실패')
            saved=capture.artifact_path
            if self.capture_stop.is_set():raise ValueError('촬영 기능 종료 중')
            job_id=self.mail.enqueue(capture_id,'current_position',saved,recipient)
            result=dict(state='QUEUED',path=saved,job_id=job_id,detail='사진 저장 완료 · 메일 발송 대기열 등록')
        except Exception as error:
            result=dict(state='FAILED',path=saved,detail=(
                '사진은 저장됐지만 메일 등록에 실패했습니다: '+type(error).__name__
                if saved else '촬영 실패: '+str(error)))
        with self.lock:self.manual_capture.update(result,busy=False)

    def _queue_photo_mail(self, location, path):
        # SMTP is handled by the worker; even local queue failures never fail motion.
        recipient=self.session.get('mail_recipient','')
        if not recipient:return
        try:
            if self.mail is None:raise ValueError('mail worker unavailable')
            self.mail.enqueue(self.session['mission_id'],location,path,recipient)
        except Exception as error:
            self.mail_error=type(error).__name__+' · 메일 대기열 저장 실패. 사진은 저장됐고 주행은 계속합니다.'

    def snapshot(self):
        with self.lock:return deepcopy(self.session)

    def save(self, **updates):
        with self.lock:
            self.session.update(updates)
            self.output.mkdir(parents=True,exist_ok=True)
            path=self.output/'session.json';temporary=path.with_suffix('.tmp')
            with temporary.open('w') as stream:
                json.dump(self.session,stream,ensure_ascii=False,indent=2);stream.flush();os.fsync(stream.fileno())
            temporary.replace(path)

    def preflight(self, returning=False):
        session=self.snapshot()
        if returning:
            if session.get('stage') not in ('WAITING_RETURN','INTERRUPTED','FAILED','CANCELLED') or not session.get('origin'):
                raise ValueError('복귀할 촬영 임무가 없습니다.')
            identity=getattr(self.executor,'photo_map_identity',None)
            if identity is not None and session.get('origin_map_sha256')!=identity(session['origin']['floor_id']):
                raise ValueError('저장된 출발 좌표의 지도가 변경되었습니다. 출발 위치를 다시 확인해야 합니다.')
        elif session.get('origin') and session.get('stage')!='COMPLETE':
            raise ValueError('기존 촬영 임무의 복귀가 남아 있습니다. 복귀 승인으로 출발점에 돌아오세요.')
        readiness=getattr(self.executor,'photo_preflight',None)
        if readiness is not None:readiness(returning=returning)

    def plan(self, planner, anchor, returning=False):
        self.preflight(returning)
        origin_id=anchor
        origin=self.snapshot().get('origin')
        targets=([origin['id']] if planner._locations[anchor].floor_id!=origin['floor_id'] else []) if returning else list(self.settings['locations'])
        segments=[]
        for target in targets:
            route=planner.plan_from_current_pose(anchor,target)
            segments.extend(json.loads(route.to_bytes())['segments']);anchor=target
            if not returning:segments.append(dict(type='PHOTO',target_id=target,source_floor='RF',profile_id=None))
        if returning:segments.append(dict(type='RETURN_POSE',target_id=origin['id'],source_floor=origin['floor_id'],profile_id=None))
        return dict(origin_id=origin_id,destination_id=origin['id'] if returning else targets[-1],segments=segments)

    def run(self, host, request, feedback, cancelled):
        returning=request.mission_type is MissionType.RETURN_TO_START
        started=False
        try:
            self.preflight(returning)
            if cancelled():
                return MissionRunResult(SegmentExecutionStatus.CANCELLED,9,"cancelled before photo dispatch",request.mission_id,"")
            if not returning:
                origin=self.executor.photo_origin(host.confirmed_location_id)
                started=True
                self.session=dict(version=1,mission_id=request.mission_id,origin=asdict(origin),photos=[],stage='OUTBOUND',mail_recipient=self.mail_recipient)
                identity=getattr(self.executor,'photo_map_identity',None)
                self.save(origin_map_sha256=identity(origin.floor_id) if identity is not None else None,detail='촬영 순회 시작')
            else:
                started=True
                self.save(stage='RETURNING',detail='사용자 승인으로 출발점 복귀 시작')
            targets=([self.session['origin']['id']] if host._planner._locations[host.confirmed_location_id].floor_id!=self.session['origin']['floor_id'] else []) if returning else self.settings['locations']
            for index,target in enumerate(targets):
                if cancelled():return self._cancel(request)
                # Reuse the canonical route executor, ownership and floor transitions.
                nav=replace(request,destination_id=target,mission_type=MissionType.NAVIGATE,return_after_task=False)
                outcome=host.run(nav,feedback,cancelled)
                if outcome.status is not SegmentExecutionStatus.SUCCESS:
                    self.save(stage='CANCELLED' if cancelled() else 'FAILED',detail=outcome.reason)
                    return outcome
                if not returning:
                    feedback(MissionProgress(request.mission_id,'RUNNING','RF','PHOTO',index,len(targets),target,'saving a fresh RGB frame'))
                    capture=self.executor.capture_photo(target,self.output/self.session['mission_id'],cancelled)
                    if capture.status is not SegmentExecutionStatus.SUCCESS:
                        self.save(stage='CANCELLED' if cancelled() else 'FAILED',detail=capture.reason)
                        return MissionRunResult(capture.status,capture.result_code,capture.reason,request.mission_id,str(self.output/self.session['mission_id']))
                    self.save(photos=self.session['photos']+[dict(location=target,path=capture.artifact_path)],detail='사진 저장: '+target)
                    self._queue_photo_mail(target,capture.artifact_path)
            if returning:
                origin=Location(**self.session['origin'])
                capture=self.executor.return_photo_origin(origin,cancelled)
                if capture.status is not SegmentExecutionStatus.SUCCESS:
                    self.save(stage='CANCELLED' if cancelled() else 'FAILED',detail=capture.reason)
                    return MissionRunResult(capture.status,capture.result_code,capture.reason,request.mission_id,'')
                from .route_planner import LogicalAnchor
                host._anchor=LogicalAnchor(origin.id)
                self.save(stage='RETURN_ARRIVED',detail='출발 위치 도착 · 위치 유지 인계 중')
            else:self.save(stage='PHOTO_ARRIVED',detail='촬영 완료 · 위치 유지 인계 중')
            return MissionRunResult(SegmentExecutionStatus.SUCCESS,0,self.session['detail'],request.mission_id,str(self.output/self.session['mission_id']))
        except Exception as error:
            if started and self.session.get('origin'):
                try:self.save(stage='CANCELLED' if cancelled() else 'FAILED',detail=str(error))
                except OSError:pass  # Preserve the original error; last durable session remains recoverable.
            return MissionRunResult(SegmentExecutionStatus.CANCELLED if cancelled() else SegmentExecutionStatus.FAILED,9 if cancelled() else 7,str(error),request.mission_id,'')

    def complete_hold(self, returning, error=None):
        if error is not None:
            self.save(stage='FAILED',detail='위치 유지 인계 실패: '+str(error))
        else:
            self.save(stage='COMPLETE' if returning else 'WAITING_RETURN',
                      detail='출발 위치 복귀 완료' if returning else '촬영 완료 · 위치 유지 · 복귀 승인 대기')

    def _cancel(self,request):
        self.save(stage='CANCELLED',detail='사용자가 촬영/복귀를 중단했습니다.')
        return MissionRunResult(SegmentExecutionStatus.CANCELLED,9,self.session['detail'],request.mission_id,'')
