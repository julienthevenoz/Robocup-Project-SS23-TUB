from naoqi import ALProxy
walkmodule = ALProxy("ALMotion", "10.0.7.113", 9559)
walkmodule.setStiffnesses("Body", 1)
walkmodule.moveInit()
walkmodule.moveTo(0, 0.5, 1)
