import smtplib
import RPi.GPIO as GPIO
import time

from picamera import PiCamera
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

SMTP_SERVER = 'smtp.gmail.com'
SMTP_PORT = 587
GMAIL_USERNAME = 'Aeeg.RasPi@gmail.com'
GMAIL_PASSWORD = 'bwmz vlam sqrh mfjr'

class Emailer:
    def sendmail(self, recipient, subject, content, image):

        #Create Headers
        emailData = MIMEMultipart()
        emailData['Subject'] = subject
        emailData['To'] = recipient
        emailData['From'] = GMAIL_USERNAME

        #Attach our text data
        emailData.attach(MIMEText(content))

        #Create our Image Data from the defined image
        imageData = MIMEImage(open(image, 'rb').read(), 'jpg')
        imageData.add_header('Content-Disposition', 'attachment; filename="image.jpg"')
        emailData.attach(imageData)

        #Connect to Gmail Server
        session = smtplib.SMTP(SMTP_SERVER, SMTP_PORT)
        session.ehlo()
        session.starttls()
        session.ehlo()

        #Login to Gmail
        session.login(GMAIL_USERNAME, GMAIL_PASSWORD)

        #Send Email & Exit
        session.sendmail(GMAIL_USERNAME, recipient, emailData.as_string())
        session.quit

sender = Emailer()

image = '/home/pi/Desktop/image001.png'

sendTo = 'andreas.eeg97@gmail.com'
emailSubject = "Button Press Detected!"
emailContent = "The button has been pressed at: " + time.ctime()
sender.sendmail(sendTo, emailSubject, emailContent, image)
print("Email Sent")
