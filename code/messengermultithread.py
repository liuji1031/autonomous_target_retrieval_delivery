import yagmail
from datetime import datetime
from threading import Thread
import time

class Messenger:
    def __init__(self, receiver=None):
        self.password = 'bibzfsfrqttdbjhn'
        self.yag = yagmail.SMTP('pi.bot.liu@gmail.com',self.password)
        self.receiver = receiver
        self.sender_name = 'Ji Liu'
        self.subject=''
        self.attachments=None
        self.do_send_now = 0
        self.stopped=0
        
    def start(self):
        print("email thread starting!")
        Thread(target=self.send,args=()).start()
    
    def send_now(self, subject, attachments):
        self.subject = subject
        self.attachments = attachments
        self.do_send_now = 1
    
    def send(self):
        while self.stopped==0:
            if self.do_send_now==1:
                self.do_send_now=0
                now = datetime.now() # current date and time
                date_time = now.strftime("%Y-%m-%d_%H-%M-%S")
                contents_ = 'Student: Ji Liu\nUID: 112960186\nDirectory ID: liuji'
                try:
                    self.yag.send(to=self.receiver,
                                subject=self.sender_name+' - '+date_time+' - '+self.subject,
                                contents=contents_,
                                cc='liuji1031@yahoo.com',
                                attachments=self.attachments)
                    print('email sent successfully')
                except:
                    print('something wrong with sending email...')
            time.sleep(1.0)
        print('email thread stopped!')
    
    def stop(self):
        self.stopped=1
