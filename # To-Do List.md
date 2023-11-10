# To-Do List
Necessary:
- Change strafing angles
- Add field oriented for both color alliances
- Figure out how to handle coordinate system for both colored alliances.
    a) One option is to convert all the coordinates and angles by turning them 180 degrees, and then run everthing based of those coordinates? I think this would work and then the semi-auton manual functions like blocking the goal only have to be implemented once. 

Test/fix:
- Test shooter code
- Test wings code
- Test gps updating
- Still drives backwards?
- Test that positive yaw vector rotates clockwise
- Think about how Tasks and while loops work with delay

Would be nice:
- Change proportional angle motor adjuster to a full PID
- Add combined strafing options in manual
- Change rotate_to_field_angle to have proportional velocity