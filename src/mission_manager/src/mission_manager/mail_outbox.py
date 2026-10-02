"""Durable photo mail worker, isolated from motion and ROS command ownership."""
from copy import deepcopy
from email.message import EmailMessage
from pathlib import Path
import hashlib,json,os,re,smtplib,ssl,threading,time


def address(value):
    if not isinstance(value,str) or len(value)>254 or not re.fullmatch(r"[A-Za-z0-9.!#$%&'*+/=?^_`{|}~-]+@[A-Za-z0-9-]+(?:\.[A-Za-z0-9-]+)+",value):
        raise ValueError('메일 주소 한 개를 입력하세요.')
    return value


class DeliveryUnknown(Exception):
    pass


class SmtpSender:
    """Local JSON settings; password is read from a separate user-owned file."""
    def __init__(self,path=None):
        self.path=Path(path or '~/.config/tron1/mail.json').expanduser()

    def settings(self):
        config=json.loads(self.path.read_text())
        required={'host','port','security','username','from_address','password_file'}
        if set(config)!=required or config['security'] not in ('ssl','starttls'):
            raise ValueError('메일 발신 설정 형식 오류')
        address(config['from_address'])
        if type(config['port']) is not int or not 1<=config['port']<=65535:
            raise ValueError('SMTP port 오류')
        return config

    def status(self):
        try:
            c=self.settings();secret=Path(c['password_file']).expanduser()
            return dict(configured=secret.is_file(),sender=c['from_address'],reason='' if secret.is_file() else '앱 비밀번호 파일 없음')
        except (OSError,ValueError,KeyError,TypeError):
            return dict(configured=False,sender='',reason='로컬 발신 계정 설정 필요')

    def __call__(self,job):
        c=self.settings();password=Path(c['password_file']).expanduser().read_text().strip()
        if not password:raise ValueError('메일 인증정보 없음')
        data=Path(job['path']).read_bytes()
        if hashlib.sha256(data).hexdigest()!=job['sha256']:
            raise ValueError('저장 사진이 변경됨')
        message=EmailMessage();message['From']=address(c['from_address']);message['To']=address(job['recipient'])
        message['Subject']='TRON 촬영 · '+job['location'];message['Message-ID']='<'+job['id']+'@tron1.local>'
        message.set_content('촬영 지점: '+job['location']+'\n미션: '+job['mission_id']+'\n저장한 RGB 사진을 첨부합니다.\n')
        message.add_attachment(data,maintype='image',subtype='jpeg',filename=Path(job['path']).name)
        context=ssl.create_default_context()
        client=None
        try:
            if c['security']=='ssl':client=smtplib.SMTP_SSL(c['host'],c['port'],timeout=12.,context=context)
            else:
                client=smtplib.SMTP(c['host'],c['port'],timeout=12.);client.ehlo();client.starttls(context=context);client.ehlo()
            client.login(c['username'],password)
            try:
                refused=client.send_message(message)
                if refused:raise smtplib.SMTPRecipientsRefused(refused)
            except (smtplib.SMTPRecipientsRefused,smtplib.SMTPResponseException):
                raise
            except (smtplib.SMTPServerDisconnected,TimeoutError,OSError) as error:
                raise DeliveryUnknown('SMTP 전송 중 연결이 끊겨 수신 여부 불명확') from error
        finally:
            # DATA acceptance means sent even if QUIT fails; do not duplicate.
            if client is not None:
                try:client.quit()
                except Exception:
                    try:client.close()
                    except Exception:pass


class MailOutbox:
    def __init__(self,photo_root,sender=None,threaded=True):
        self.root=Path(photo_root).resolve();self.file=self.root/'mail-outbox.json'
        self.sender=sender or SmtpSender();self.lock=threading.RLock();self.wake=threading.Event();self.stop=threading.Event()
        self.error='';self.thread=None;self.storage_ok=True
        try:
            self.jobs=json.loads(self.file.read_text()) if self.file.exists() else {}
            if not isinstance(self.jobs,dict) or any(not self._valid_job(k,j) for k,j in self.jobs.items()):
                raise ValueError('invalid outbox history')
        except (OSError,ValueError,TypeError):
            self.jobs={};self.storage_ok=False;self.error='발송 기록을 읽을 수 없음 · 기존 파일을 보존했습니다.'
        for job in self.jobs.values():
            if job['state']=='SENDING':job.update(state='UNKNOWN',error='발송 중 재시작됨 · 수신함 확인 후 재전송')
        if threaded:
            self.thread=threading.Thread(target=self._loop,name='photo-mail-outbox',daemon=True);self.thread.start()

    def _valid_job(self,key,job):
        try:
            required={'id','mission_id','location','path','recipient','sha256','state','attempts','error','created_at'}
            if not isinstance(job,dict) or not required<=set(job):return False
            if key!=job['id'] or not re.fullmatch('[0-9a-f]{64}',key):return False
            if not re.fullmatch('[0-9a-f]{64}',job['sha256']):return False
            if job['state'] not in ('QUEUED','SENDING','SENT','FAILED','UNKNOWN'):return False
            if type(job['attempts']) is not int or job['attempts']<0:return False
            if not all(isinstance(job[k],str) for k in ('mission_id','location','path','error')):return False
            address(job['recipient']);Path(job['path']).resolve().relative_to(self.root)
            return True
        except (TypeError,ValueError,KeyError):return False

    def _save(self):
        if not self.storage_ok:raise ValueError('mail history unavailable')
        self.root.mkdir(parents=True,exist_ok=True);temp=self.file.with_suffix('.tmp')
        with os.fdopen(os.open(str(temp),os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600),'w') as f:json.dump(self.jobs,f,ensure_ascii=False,indent=2);f.flush();os.fsync(f.fileno())
        temp.replace(self.file)

    def enqueue(self,mission_id,location,path,recipient):
        if not self.storage_ok:raise ValueError('mail history unavailable')
        recipient=address(recipient);path=Path(path).resolve()
        try:path.relative_to(self.root)
        except ValueError:raise ValueError('사진 저장소 외부 첨부 거절')
        data=path.read_bytes();digest=hashlib.sha256(data).hexdigest()
        key=hashlib.sha256(json.dumps([mission_id,location,digest,recipient]).encode()).hexdigest()
        with self.lock:
            if key not in self.jobs:
                self.jobs[key]=dict(id=key,mission_id=mission_id,location=location,path=str(path),recipient=recipient,
                    sha256=digest,state='QUEUED',attempts=0,error='',created_at=time.time())
                try:self._save()
                except Exception:
                    del self.jobs[key]
                    raise
        self.wake.set();return key

    def retry(self,key,confirm_unknown=False):
        with self.lock:
            if key not in self.jobs:raise ValueError('발송 기록 없음')
            job=self.jobs[key]
            if job['state']=='UNKNOWN' and not confirm_unknown:raise ValueError('수신 여부가 불명확합니다. 수신함 확인 후 재전송을 승인하세요.')
            if job['state'] not in ('FAILED','UNKNOWN'):raise ValueError('재전송 가능한 실패 기록이 아닙니다.')
            previous=deepcopy(job);job.update(state='QUEUED',error='')
            try:self._save()
            except Exception:
                job.clear();job.update(previous);raise
        self.wake.set();return dict(queued=True)

    def tick(self):
        with self.lock:
            job=next((j for j in self.jobs.values() if j['state']=='QUEUED'),None)
            if job is None:return
            job.update(state='SENDING',attempts=job['attempts']+1)
            try:self._save()
            except Exception:
                job.update(state='FAILED',error='발송 전 기록 저장 실패 · 메일은 보내지 않았습니다.')
                raise
            work=deepcopy(job)
        try:self.sender(work);state,error='SENT',''
        except DeliveryUnknown:state,error='UNKNOWN','서버 수신 여부 불명확 · 수신함 확인 후 재전송'
        except Exception as exc:
            # Never expose SMTP credentials or server-returned private content.
            state,error='FAILED',type(exc).__name__+' · 발신 설정/연결 확인 후 재전송'
        with self.lock:
            job.update(state=state,error=error,finished_at=time.time());self._save()

    def snapshot(self):
        with self.lock:
            values=[{k:v for k,v in j.items() if k not in ('path','sha256')} for j in self.jobs.values()]
            settings=self.sender.status() if hasattr(self.sender,'status') else dict(configured=True,sender='test')
            return dict(**settings,jobs=deepcopy(values[-100:]),worker_error=self.error)

    def _loop(self):
        while not self.stop.is_set():
            try:
                if self.storage_ok:self.tick();self.error=''
            except Exception as exc:self.error=type(exc).__name__+' · 발송 기록 저장 실패'
            self.wake.wait(.5);self.wake.clear()

    def close(self):
        self.stop.set();self.wake.set()
        if self.thread:self.thread.join(timeout=1.)
