from naoqi import ALProxy
speakingmodule = ALProxy("ALTextToSpeech", "10.0.7.113", 9559)
speakingmodule.say("Hi Julien")
