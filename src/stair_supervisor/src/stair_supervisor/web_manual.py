"""An expiring manual input lease. Caller holds the transport command lock."""
import math
import uuid
from .robot_conversion import NormalizedTwist


class WebManual:
    timeout_sec=.35

    def __init__(self):
        self.lease=None
        self.sequence=-1
        self.expires=0.
        self.ended=True
        self.command=NormalizedTwist.zero()

    def begin(self,now):
        if self.lease and not self.ended and now<self.expires:
            raise ValueError('다른 수동 조작이 진행 중입니다.')
        self.lease=uuid.uuid4().hex
        self.sequence=-1;self.expires=now+self.timeout_sec;self.ended=False
        self.command=NormalizedTwist.zero()
        return self.lease

    def update(self,lease,sequence,forward,turn,now,remaining):
        if lease!=self.lease or self.ended or now>=self.expires:
            raise ValueError('수동 입력이 만료됐습니다. 조이스틱을 놓고 다시 잡으세요.')
        if type(sequence) is not int or sequence<=self.sequence:
            raise ValueError('이전 수동 입력 거부')
        if any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>1 for v in (forward,turn)):
            raise ValueError('잘못된 조이스틱 값')
        if not 0<remaining<=self.timeout_sec:
            raise ValueError('지연된 조이스틱 입력')
        self.sequence=sequence;self.expires=now+remaining
        self.command=NormalizedTwist(float(forward),0.,float(turn))

    def end(self,lease):
        if lease!=self.lease:raise ValueError('수동 제어 세션 불일치')
        self.ended=True;self.expires=0.;self.command=NormalizedTwist.zero()

    def release(self):
        self.lease=None;self.ended=True;self.expires=0.;self.command=NormalizedTwist.zero()

    def selected(self,now):
        if self.lease is None or self.ended or now>=self.expires:return None
        return self.command

    def snapshot(self,now):
        return dict(active=self.lease is not None and not self.ended and now<self.expires,
                    sequence=self.sequence,timeout_sec=self.timeout_sec)
