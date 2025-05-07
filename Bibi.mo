model Bibi
  inner Modelica.Mechanics.MultiBody.World world(animateGround = true, n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.RollingWheel wheel(radius(displayUnit = "mm") = 0.065, m = 0.1, I_axis = 0.00001, I_long = 0.00001, width(displayUnit = "mm") = 0.0725, hollowFraction = 0.8, x(start = 0), y(start = 0), der_angles(each fixed = false), angles(each fixed = false))  annotation(
    Placement(transformation(origin = {-30, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute( n = {0, 1, 0}, cylinderLength = 0.1, cylinderDiameter(displayUnit = "mm") = 0.005, phi(start = 0.7853981633974483, fixed = true), useAxisFlange = true)  annotation(
    Placement(transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyBox bodyBox(r = {0.025, 0, 0}, density = 100)  annotation(
    Placement(transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.PointMass pointMass(m = 0.1, sphereDiameter = 0.01)  annotation(
    Placement(transformation(origin = {80, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Components.Damper damper(d = 0.0001)  annotation(
    Placement(transformation(origin = {10, 80}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(revolute.frame_a, wheel.frame_a) annotation(
    Line(points = {{0, 50}, {-30, 50}}, color = {95, 95, 95}));
  connect(revolute.frame_b, bodyBox.frame_a) annotation(
    Line(points = {{20, 50}, {40, 50}}, color = {95, 95, 95}));
  connect(bodyBox.frame_b, pointMass.frame_a) annotation(
    Line(points = {{60, 50}, {80, 50}}, color = {95, 95, 95}));
  connect(revolute.axis, damper.flange_b) annotation(
    Line(points = {{10, 60}, {20, 60}, {20, 80}}));
  connect(revolute.support, damper.flange_a) annotation(
    Line(points = {{4, 60}, {0, 60}, {0, 80}}));
  annotation(
    uses(Modelica(version = "4.0.0")));
end Bibi;
