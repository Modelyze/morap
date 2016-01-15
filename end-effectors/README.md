# Different End Effectors Developed for the Robotic Arm

Contains different end-effectors capabale of mounting to the arm and interfacing with software.

## Making you own
**IMPORTANT. WHEN CONNECTING END EFFECTORS TO THE DATA BUS THE I2C LINES NEEDS TO WORK ON A 3.3 VOLT VOLTAGE LEVEL. OTHERWISE THE MOTOR CONTROLLING MICROCHIPS WILL BE FRIED.**  

This can be done through using chip that works on a 3.3 volt voltage level or using level shifters.

## Testing
| return value | meaning |
| ---: | :--- |
| 1 | nothing has been programmed |
| 2 | programming successful |
| 3 | programming failed |

return value | meaning 
---: | :--- 
1 | nothing has been programmed 
2 | programming successful 
3 | programming failed 
 
return value | meaning 
----------: | :------------- 
1 | nothing has been programmed 
2 | programming successful 
3 | programming failed 


Markdown | Less | Pretty
------------- | ------------- | -----------
*Still* | `renders` | **nicely**
1 | 2 | 3