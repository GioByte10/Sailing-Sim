#SingleInstance, force
ClickOn(x,y) {
MouseMove, x, y
sleep 50
Click
}

+^a:: ; Press bias sensor
ClickOn(47,407)
return

+^b::
ClickOn(101,404) ; Press unbias sensor
return

+^c:: ; Press stop sensor log
ClickOn(72,494)
return

+^d:: ; Press start sensor log
ClickOn(72,494)
return



+^f::
MouseGetPos, xpos, ypos 
MsgBox, %xpos% %ypos%
return


