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
                stage=value.get('stage')
                missed=value.get('missed_photos',[])
                outcome=('PARTIAL' if missed else 'INCOMPLETE') if stage=='OUTBOUND' else ('PARTIAL' if missed else 'COMPLETE')
                self.session.update(stage='INTERRUPTED',photo_outcome=outcome,detail='실행 중 재시작됨 · 자동 재개하지 않음')

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

    def _queue_photo_mail(self, location, path, visit_index=None):
        # SMTP is handled by the worker; even local queue failures never fail motion.
        recipient=self.session.get('mail_recipient','')
        if not recipient:return
        try:
            if self.mail is None:raise ValueError('mail worker unavailable')
            mail_id=self.session['mission_id']
            if visit_index is not None and location in self.settings['locations'][:visit_index]:
                mail_id+=':visit:'+str(visit_index)
            self.mail.enqueue(mail_id,location,path,recipient)
        except Exception as error:
            self.mail_error=type(error).__name__+' · 메일 대기열 저장 실패. 사진은 저장됐고 주행은 계속합니다.'

    def _remaining_visits(self, session):
        # A location may occur twice; completion belongs to a route occurrence.
        photos=[item for item in session.get('photos',[]) if isinstance(item,dict)]
        indexed={item['visit_index'] for item in photos if 'visit_index' in item}
        legacy=[item.get('location') for item in photos if 'visit_index' not in item]
        remaining=[]
        for index,location in enumerate(self.settings['locations']):
            if index in indexed:continue
            if location in legacy:
                legacy.remove(location)
                continue
            remaining.append((index,location))
        return remaining

    def _remaining_locations(self, session):
        return [location for _,location in self._remaining_visits(session)]

    def _route_matches(self, session):
        route=session.get('route_locations')
        if route is not None:return route == list(self.settings['locations'])
        return len(set(self.settings['locations'])) == len(self.settings['locations'])

    def _retry_available(self, session):
        return bool(session.get('origin') and self._route_matches(session) and
                    session.get('stage') in ('FAILED','CANCELLED','INTERRUPTED') and
                    not session.get('return_started',False) and
                    self._remaining_locations(session))

    def snapshot(self):
        with self.lock:
            state=deepcopy(self.session)
            if state:state['retry_available']=self._retry_available(state)
            return state

    def save(self, **updates):
        with self.lock:
            self.session.update(updates)
            self.output.mkdir(parents=True,exist_ok=True)
            path=self.output/'session.json';temporary=path.with_suffix('.tmp')
            with temporary.open('w') as stream:
                json.dump(self.session,stream,ensure_ascii=False,indent=2);stream.flush();os.fsync(stream.fileno())
            temporary.replace(path)

    def preflight(self, returning=False, rooftop_only=False):
        session=self.snapshot()
        if returning:
            if session.get('stage') not in ('WAITING_RETURN','INTERRUPTED','FAILED','CANCELLED') or not session.get('origin'):
                raise ValueError('복귀할 촬영 임무가 없습니다.')
            identity=getattr(self.executor,'photo_map_identity',None)
            if identity is not None and session.get('origin_map_sha256')!=identity(session['origin']['floor_id']):
                raise ValueError('저장된 출발 좌표의 지도가 변경되었습니다. 출발 위치를 다시 확인해야 합니다.')
        elif not rooftop_only and self._route_matches(session) and session.get('origin') and session.get('stage')!='COMPLETE':
            if not self._retry_available(session):
                raise ValueError('기존 촬영 임무의 복귀가 남아 있습니다. 복귀 승인으로 출발점에 돌아오세요.')
            identity=getattr(self.executor,'photo_map_identity',None)
            if identity is not None and session.get('origin_map_sha256')!=identity(session['origin']['floor_id']):
                raise ValueError('저장된 출발 좌표의 지도가 변경되었습니다. 출발 위치를 다시 확인해야 합니다.')
        readiness=getattr(self.executor,'photo_preflight',None)
        if readiness is not None:readiness(returning=returning)

    def plan(self, planner, anchor, returning=False, rooftop_only=False):
        self.preflight(returning, rooftop_only)
        if rooftop_only and planner._locations[anchor].floor_id != 'RF':
            raise ValueError('옥상 순회는 RF 지도와 현재 위치 확인 후 시작합니다.')
        origin_id=anchor
        session=self.snapshot()
        origin=session.get('origin')
        targets=([origin['id']] if planner._locations[anchor].floor_id!=origin['floor_id'] else []) if returning else (self._remaining_locations(session) if self._retry_available(session) else list(self.settings['locations']))
        segments=[]
        for target in targets:
            route=planner.plan_from_current_pose(anchor,target)
            if rooftop_only and any(s.source_floor != 'RF' or s.target_floor != 'RF' for s in route.segments):
                raise ValueError('옥상 순회에 다른 층 경로를 포함할 수 없습니다.')
            segments.extend(json.loads(route.to_bytes())['segments']);anchor=target
            if not returning:segments.append(dict(type='PHOTO',target_id=target,source_floor='RF',profile_id=None))
        if returning:segments.append(dict(type='RETURN_POSE',target_id=origin['id'],source_floor=origin['floor_id'],profile_id=None))
        return dict(origin_id=origin_id,destination_id=origin['id'] if returning else targets[-1],segments=segments)

    def run(self, host, request, feedback, cancelled):
        returning=request.mission_type is MissionType.RETURN_TO_START
        rooftop_only=request.mission_type is MissionType.ROOFTOP_TOUR
        started=False
        try:
            self.preflight(returning, rooftop_only)
            if rooftop_only and host._planner._locations[host.confirmed_location_id].floor_id != 'RF':
                raise ValueError('옥상 순회는 RF 지도와 현재 위치 확인 후 시작합니다.')
            if cancelled():
                return MissionRunResult(SegmentExecutionStatus.CANCELLED,9,"cancelled before photo dispatch",request.mission_id,"")
            if rooftop_only:
                self.executor.prepare_rooftop_navigation()
                if cancelled():
                    return self._cancel(request)
            if not returning:
                if self._retry_available(self.snapshot()):
                    started=True
                    self.save(stage='OUTBOUND',return_started=False,
                              photo_outcome='PARTIAL' if self.session.get('missed_photos') else 'IN_PROGRESS',
                              detail='현재 위치에서 남은 촬영 지점 재시도')
                else:
                    origin=self.executor.photo_origin(host.confirmed_location_id)
                    if self.session:
                        archive=self.output/self.session['mission_id']/'session.json'
                        archive.parent.mkdir(parents=True,exist_ok=True)
                        archive.write_text(json.dumps(self.session,ensure_ascii=False,indent=2))
                    started=True
                    self.session=dict(version=1,mission_id=request.mission_id,origin=asdict(origin),route_locations=list(self.settings['locations']),photos=[],missed_photos=[],photo_outcome='IN_PROGRESS',stage='OUTBOUND',mail_recipient=self.mail_recipient,return_started=False)
                    identity=getattr(self.executor,'photo_map_identity',None)
                    self.save(origin_map_sha256=identity(origin.floor_id) if identity is not None else None,detail='촬영 순회 시작')
            else:
                started=True
                self.save(stage='RETURNING',return_started=True,detail='사용자 승인으로 출발점 복귀 시작')
            targets=([self.session['origin']['id']] if host._planner._locations[host.confirmed_location_id].floor_id!=self.session['origin']['floor_id'] else []) if returning else self._remaining_locations(self.session)
            visits=list(enumerate(targets)) if returning else self._remaining_visits(self.session)
            for index,target in visits:
                if cancelled():return self._cancel(request)
                # Reuse the canonical route executor, ownership and floor transitions.
                nav=replace(request,destination_id=target,mission_type=MissionType.NAVIGATE,return_after_task=False)
                outcome=host.run(nav,feedback,cancelled)
                if outcome.status is not SegmentExecutionStatus.SUCCESS:
                    self.save(stage='CANCELLED' if cancelled() else 'FAILED',detail=outcome.reason)
                    return outcome
                if not returning:
                    feedback(MissionProgress(request.mission_id,'RUNNING','RF','PHOTO',index,len(targets),target,'saving a fresh RGB frame'))
                    try:
                        capture=self.executor.capture_photo(target,self.output/self.session['mission_id'],cancelled)
                    except Exception as error:
                        capture=None
                        capture_error='{}: {}'.format(type(error).__name__,error)
                    else:
                        capture_error=''
                    if capture is not None and capture.status is SegmentExecutionStatus.CANCELLED:
                        return self._cancel(request)
                    if cancelled():
                        return self._cancel(request)
                    if capture is None or capture.status is not SegmentExecutionStatus.SUCCESS:
                        reason=capture_error or (capture.reason if capture is not None else '사진 촬영 실패')
                        missed=[item for item in self.session.get('missed_photos',[])
                                if item.get('visit_index', -1)!=index]
                        missed.append(dict(location=target,visit_index=index,reason=reason,
                                           result_code=int(capture.result_code) if capture is not None else 7))
                        self.save(missed_photos=missed,photo_outcome='PARTIAL',
                                  detail='촬영 누락: {} · 다음 지점으로 이동'.format(target))
                        continue
                    missed=[item for item in self.session.get('missed_photos',[])
                            if item.get('visit_index', -1)!=index]
                    self.save(photos=self.session['photos']+[dict(location=target,visit_index=index,path=capture.artifact_path)],
                              missed_photos=missed,
                              photo_outcome='PARTIAL' if missed else 'IN_PROGRESS',
                              detail='사진 저장: '+target)
                    self._queue_photo_mail(target,capture.artifact_path,visit_index=index)
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
        photos=self.session.get('photos',[])
        missed=self.session.get('missed_photos',[])
        total=len(photos)+len(missed)
        outcome='PARTIAL' if missed else 'COMPLETE'
        if missed:
            points=', '.join(item.get('location','?') for item in missed)
            summary='촬영 일부 누락 ({}/{}개 성공): {}'.format(len(photos),total,points)
        else:
            summary='촬영 완료 ({}/{}개)'.format(len(photos),total)
        if error is not None:
            detail='위치 유지 인계 실패: {} · {}'.format(error,summary)
            self.save(stage='FAILED',photo_outcome=outcome,detail=detail)
            return
        detail=('출발 위치 복귀 완료 · '+summary if returning else
                summary+' · 위치 유지 · 복귀 승인 대기')
        self.save(stage='COMPLETE' if returning else 'WAITING_RETURN',
                  photo_outcome=outcome,detail=detail)

    def _cancel(self,request):
        missed=self.session.get('missed_photos',[])
        outcome=self.session.get('photo_outcome','INCOMPLETE')
        if request.mission_type in (MissionType.PHOTO_TOUR, MissionType.ROOFTOP_TOUR):
            outcome='PARTIAL' if missed else 'INCOMPLETE'
        detail='사용자가 촬영/복귀를 중단했습니다.'
        if missed:
            detail+=' · 촬영 일부 누락: '+', '.join(item.get('location','?') for item in missed)
        self.save(stage='CANCELLED',photo_outcome=outcome,detail=detail)
        return MissionRunResult(SegmentExecutionStatus.CANCELLED,9,self.session['detail'],request.mission_id,'')
