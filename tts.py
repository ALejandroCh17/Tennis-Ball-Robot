import sys
from robot_hat import TTS

if len(sys.argv) > 1:
    text_to_say = sys.argv[1]
else:
    text_to_say = "Hello SunFounder"

tts = TTS(lang='en-US')
tts.say(text_to_say)
print(tts.supported_lang())
