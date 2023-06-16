from naoqi import ALProxy
speakingmodule = ALProxy("ALTextToSpeech", "10.0.7.100", 9559)
speakingmodule.say("Hello, world! Julien is here")
