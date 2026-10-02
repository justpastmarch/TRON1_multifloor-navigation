"""One fresh RGB photograph per stop; owns no ROS node or motion source."""
import threading,time,os
from pathlib import Path
from .mission_types import SegmentExecution


class PhotoCapture:
    def __init__(self, topic, timeout):
        import rospy
        from sensor_msgs.msg import CompressedImage
        self.condition=threading.Condition();self.latest=None;self.sequence=0
        self.timeout=timeout
        self.subscriber=rospy.Subscriber(topic,CompressedImage,self.receive,queue_size=1)
        rospy.on_shutdown(self.subscriber.unregister)

    def receive(self,message):
        with self.condition:
            self.sequence+=1;self.latest=(message,time.monotonic(),self.sequence)
            self.condition.notify_all()

    def readiness(self):
        import rospy
        with self.condition:
            value=self.latest
            if value is None:return False
            message,received,_=value
            return (0<=rospy.Time.now().to_sec()-message.header.stamp.to_sec()<=2.
                    and 0<=time.monotonic()-received<=1.)

    def capture(self, location, output, cancelled):
        import rospy,cv2,numpy as np
        capture_stamp=rospy.Time.now().to_nsec()
        deadline=time.monotonic()+self.timeout
        with self.condition:
            fence=self.sequence
            while True:
                if cancelled():return SegmentExecution.cancelled('photo cancelled')
                if time.monotonic()>=deadline:return SegmentExecution.failed(7,'fresh RGB frame timeout')
                value=self.latest
                if value and value[2]>fence:
                    message,received,_=value
                    age=rospy.Time.now().to_sec()-message.header.stamp.to_sec()
                    if message.header.stamp.to_nsec()>capture_stamp and 0<=age<=2. and time.monotonic()-received<=1.:
                        break
                self.condition.wait(.05)
        image=cv2.imdecode(np.frombuffer(message.data,dtype=np.uint8),cv2.IMREAD_COLOR)
        if image is None:return SegmentExecution.failed(7,'RGB image decode failed')
        ok,data=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,95])
        if not ok:return SegmentExecution.failed(7,'JPEG encoding failed')
        if cancelled():return SegmentExecution.cancelled('photo cancelled before save')
        output=Path(output);output.mkdir(parents=True,exist_ok=True)
        target=output/(location+'_'+str(message.header.stamp.to_nsec())+'.jpg')
        temp=target.with_suffix('.part')
        try:
            with temp.open('wb') as f:f.write(data.tobytes());f.flush();os.fsync(f.fileno())
            if cancelled():return SegmentExecution.cancelled('photo cancelled before commit')
            temp.replace(target)
        finally:
            if temp.exists():temp.unlink()
        return SegmentExecution.success(str(target),'fresh RGB JPEG saved')
