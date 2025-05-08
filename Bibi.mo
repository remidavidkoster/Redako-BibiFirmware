model Bibi
  inner Modelica.Mechanics.MultiBody.World world(animateGround = true, n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {70, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.RollingWheel wheel(radius(displayUnit = "mm") = 0.065, m = 0.1, I_axis = 0.00001, I_long = 0.00001, width(displayUnit = "mm") = 0.0725, hollowFraction = 0.8, x(start = 0), y(start = 0), der_angles(each fixed = false), angles(each fixed = true))  annotation(
    Placement(transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute( n = {0, 1, 0}, cylinderLength = 0.1, cylinderDiameter(displayUnit = "mm") = 0.005, useAxisFlange = true)  annotation(
    Placement(transformation(origin = {-10, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.PointMass pointMass(m = 0.1, sphereDiameter = 0.01) annotation(
    Placement(transformation(origin = {70, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sources.Position position(useSupport = true, final exact = true)  annotation(
    Placement(transformation(origin = {-10, 38}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodyCylinder(r = {0, 0, -0.025})  annotation(
    Placement(transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.KinematicPTP2 kinematicPTP(q_end = {5}, qd_max = {5}, qdd_max = {2}) annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(revolute.frame_a, wheel.frame_a) annotation(
    Line(points = {{-20, -10}, {-50, -10}}, color = {95, 95, 95}));
  connect(revolute.axis, position.support) annotation(
    Line(points = {{-10, 0}, {-10, 28}}));
  connect(revolute.support, position.flange) annotation(
    Line(points = {{-16, 0}, {-14, 0}, {-14, 10}, {0, 10}, {0, 38}}));
  connect(bodyCylinder.frame_b, pointMass.frame_a) annotation(
    Line(points = {{40, -10}, {70, -10}}, color = {95, 95, 95}));
  connect(kinematicPTP.q[1], position.phi_ref) annotation(
    Line(points = {{-59, 38}, {-22, 38}}, color = {0, 0, 127}));
  connect(bodyCylinder.frame_a, revolute.frame_b) annotation(
    Line(points = {{20, -10}, {0, -10}}, color = {95, 95, 95}));
  annotation(
    uses(Modelica(version = "4.0.0")));
end Bibi;
