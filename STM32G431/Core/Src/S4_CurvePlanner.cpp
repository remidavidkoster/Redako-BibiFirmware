//#include <Arduino.h>
#include <S4_CurvePlanner.h>
#include <math.h>
#include <main.h>

#define min4f(a, b, c, d) fminf(fminf(a, b), fminf(c, d))
#define min3f(a, b, c) fminf(fminf(a, b), c)

//#define __debug

S4_CurvePlanner::S4_CurvePlanner(int tickPeriod){
	plannerPeriod = tickPeriod;
	isTrajectoryExecuting = false;
	return;
}

//void S4_CurvePlanner::linkMotor(FOCMotor *motorReference){
//    motor = motorReference;
//}

bool S4_CurvePlanner::isPlannerMoving(){
	return isTrajectoryExecuting;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){ 
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//
//void S4_CurvePlanner::doMCommand(char *MCommand){

//
//    /***********************************************************************************
//
//    //Variables for calculating pos_target using deltaTime, velocity and acceleration.
//    float Last_position;
//    float last_velocity;
//    float last_acceleratio;
//    float now_;
//    float deltaTime;
//
//    *************************************************************************************/
//
//
//    //The controller can report axes positions, including extra axes (A, B, C etc.), typically with the M114 command.
//
//
//        if (commandValue_Int == 114){
//        SerialUSB.println("ok");
//        // M114
//        // Send current position
//
//        //MM per revolution movement
//        //int mm_per_rev = 17;
//
//        //convert to mm from 2 x radians per revolution using the mm_per_rev variable as the mm per revolution
//
//        float mm = mapfloat(motor->shaft_angle, 0, 2*PI, 0, buffer.mm_per_rev);
//        SerialUSB.print("X:");
//        SerialUSB.println(mm, 4); }
//
//
//
//        // The controller must be able to wait for motion completion, typically with the M400 command. Any further commands sent after the M400 must be suspended until motion completion.
//        // The controller must only acknowledge the command, when motion is complete i.e. the "ok" (COMMAND_CONFIRM_REGEX) response must be suspended until then, providing blocking
//        // synchronization to OpenPnP.
//
//        if (commandValue_Int == 400){
//            // M400
//            // Wait for current move to complete
//            m400_flag = true;
//            // SerialUSB.println("ok");
//            }
//
//        //The controller must support dynamic acceleration and/or jerk limits, typically by the M204 command for acceleration or the M201.3 command for jerk.
//        float commandValue2;
//
//            if (commandValue_Int == 204){
//            // M204
//            // Set acceleration
//
//        commandSrt = commandSrt.substring(4);
//        commandValue2 = commandSrt.toFloat();
//        vmax = commandValue2;
//        // Try calling the planner to use this new velocity value
//        // We have to use the current pos, vel, and accel
//        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
//        //calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
//        #ifdef __debug
//        SerialUSB.print("User wants velocity change. Vmax_: ");
//        SerialUSB.println(vmax);
//        #endif
//            }
//
//
//        if (commandValue_Int == 208){
//        //Note: Changed to 208 to avoid colision with 201(201.3) command´s
//        // M201.3
//        // Set jerk
//
//        commandSrt = commandSrt.substring(4);
//        commandValue2 = commandSrt.toFloat();
//        _Jmax_= commandValue2;
//
//
//        // Try calling the planner to use this new acceleration value
//        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
//        //calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
//
//        #ifdef __debug
//        SerialUSB.print("JERK_MAX: ");
//        SerialUSB.println(_Jmax_);
//        #endif
//
//            }
//
//    // The controller must be able to home.
//    //The homing procedure must be configurable, typically with the M206 command.
//    //The controller must be able to home to a position other than 0.
//
//        if (commandValue_Int == 206){
//        // M206
//        // Set current position, for homing to 0.
//
//        commandSrt = commandSrt.substring(4);
//        commandValue2 = commandSrt.toFloat();
//
//        motor->shaft_angle = commandValue2;
//
//        // NOTE: Probably need to set position in a better mannor.
//
//        SerialUSB.print("HOMING_TO:");
//        SerialUSB.println(commandValue2);
//
//        SerialUSB.print("POS:");
//        SerialUSB.println(motor->shaft_angle); }
//
//
//
//        if (commandValue_Int == 207){
//        // M207 - NEW M-COMMAND
//        // Set MAX SNAP
//
//        commandSrt = commandSrt.substring(4);
//        commandValue2 = commandSrt.toFloat();
//        _Smax_= commandValue2;
//
//        #ifdef __debug
//        SerialUSB.print("SNAP_MAX: ");
//        SerialUSB.println(_Smax_);
//        #endif
//
//          }
//
//
//
//    if (commandValue_Int == 203){
//        // M203
//        // Set maximum feedrate/velocity
//        commandSrt = commandSrt.substring(4);
//        commandValue2 = commandSrt.toFloat();
//        _Vmax_= commandValue2;
//
//        #ifdef __debug
//        SerialUSB.print("VEL_MAX: ");
//        SerialUSB.println(_Jmax_);
//        #endif
//
//    }
//
//
//    if (commandValue_Int == 201){
//        // M201
//        // Set maximum acceleration
//        commandSrt = commandSrt.substring(4);
//        commandValue2 = commandSrt.toFloat();
//        _Amax_= commandValue2;
//
//        #ifdef __debug
//        SerialUSB.print("ACCEL_MAX: ");
//        SerialUSB.println(_Jmax_);
//        #endif
//
//
//
//    }
//
//
//    if (commandValue_Int == 202){
//        // M202
//        // Set maximum jerk
//        commandSrt = commandSrt.substring(4);
//        commandValue2 = commandSrt.toFloat();
//        _Jmax_= commandValue2;
//
//        #ifdef __debug
//        SerialUSB.print("JERK_MAX: ");
//        SerialUSB.println(_Jmax_);
//        #endif
//    }
//
//     if (commandValue_Int == 425){
//        // M425
//        // Backlash compensation
//        // Note: Should not be needed with sense on shaft...Perhaps usefull if using belts or spring loaded screw w. tiny amount of slag
//        // This is outside the scope of this theoretical perfect move, since the machine constraints and physics may have some minute deviance.
//        // This deviance, between perfect move and actual move, can of cource add up over time, not to slag!
//        // Sure, with vision this acumulativ error get zeroed out, when homing to fiducals or end-stop actuator/sensor. Fixed machine head/camera/vision/sensor pointers.
//
//    }
//
//    // https://snapmaker.github.io/Documentation/gcode/M425
//
//
//    if (commandValue_Int == 205){
//        // M205
//        // Advanced settings
//
//
//    }
//
//
///*
//Note: TODO?
//M205 - Set Advanced Settings
//Description
//Set various motion settings. See parameters for details.
//
//Notes
//View the current setting with M503.
//
//....
//
//https://snapmaker.github.io/Documentation/gcode/M425
//
//
//*/
//
//
///*
//        #ifdef __debug
//            SerialUSB.print("M Command :  ");
//            SerialUSB.println(commandValue_Int);
//        #endif
//
//*/
//
//}



//void S4_CurvePlanner::executeCommand(char *gCodeCommand, char *gCodeCommand2){
//
//    #ifdef __debug
//        SerialUSB.print("GGode command: G");
//        SerialUSB.println(gCodeCommand);
//    #endif
//
//    //Serial.println("ok");
//
//    // Parse this string for vals
////    String commandSrt = String(gCodeCommand);
////    commandSrt = commandSrt.substring(1);
//    float commandValue;
//    switch (gCodeCommand[0]){
//
//
//    case 'V':
//    // Remove V so can convert to a float
//    commandSrt = commandSrt.substring(1);
//    commandValue = commandSrt.toFloat();
//    _Vmax_ = commandValue;
//    // Try calling the planner to use this new velocity value
//    // We have to use the current pos, vel, and accel
//    // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
//    //calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
//    //This new value will be used when the gCommand is executed.
//    #ifdef __debug
//        SerialUSB.print("User wants velocity change. Vmax_: ");
//        SerialUSB.println(vmax);
//    #endif
//    break;
//
//    case 'A':
//    // Remove A so can convert to a float
//    commandSrt = commandSrt.substring(1);
//    commandValue = commandSrt.toFloat();
//    _Amax_ = commandValue;
//    //Dmax_ = Amax_;
//    // Try calling the planner to use this new acceleration value
//    // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
//    //calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
//    //This new value will be used when the gCommand is executed.
//    #ifdef __debug
//        SerialUSB.print("User wants acceleration change. Amax_: ");
//        SerialUSB.println(amax);
//
//        #endif
//        break;
//
//    case 'M':
//    commandSrt = commandSrt.substring(1);
//    //commandValue = commandSrt.toFloat();
//    commandSrt.toCharArray(command_char, 10);
//    doMCommand(command_char);
//
//        break;
//
//    default:
//    // Remove G so can convert to a float
//    commandSrt = commandSrt.substring(1);
//    commandValue = commandSrt.toFloat();
//
//    //SerialUSB.print("Float: ");
//        //  SerialUSB.println(commandValue, 5);
//        //=  map(commandValue, 0, buffer.mm_per_rev, 0, 2*PI);
//
//    float angle_command = mapfloat(commandValue, 0, buffer.mm_per_rev, 0, 2*PI);
//
//    #ifdef __debug
//    SerialUSB.print("Move to new position (rads): ");
//    SerialUSB.println(angle_command, 5);
//    #endif
//
//    // We start moving to the new position
//    Initiate_Move(angle_command);
//    break;
//    }
//
//}




void S4_CurvePlanner::runPlannerOnTick(){
	// This could run 10 000 times / second (10kHz) ?
	//  https://www.youtube.com/watch?v=EYWEprqoLAQ

	if ((unsigned long)(TIM2->CNT - plannerTimeStep) > plannerPeriod){
		plannerTimeStep = TIM2->CNT;

		float timeSinceStartingTrajectoryInSeconds;

		// see if we are in a move or not
		if (isTrajectoryExecuting){
			// we are in a move, let's calc the next position
			timeSinceStartingTrajectoryInSeconds = ((TIM2->CNT) - plannerStartingMovementTimeStamp) / 1000000.0f;
			RuntimePlanner(timeSinceStartingTrajectoryInSeconds);
			// motor->target = vel_target;
		}

		// see if we are done with our move
		if (timeSinceStartingTrajectoryInSeconds >= Tf){
			// we are done with move

			isTrajectoryExecuting = false;
		}
	}
}



void S4_CurvePlanner::CalculateTs(float Ts_vmax, float Ts_amax, float Ts_jmax) {

	// Step 1: Calculate Ts from displacement constraint
	float ds_max = dmax / (8 * smax);
	Ts_d = pow(ds_max, 0.25f);

	//vmax = (2 * pow(Ts, 3)) * smax;
	// Step 2: Calculate Ts from velocity constraint
	Ts_v = cbrt(Ts_vmax / (2 * smax));

	// Step 3: Calculate Ts from acceleration constraint
	//amax = pow(Ts, 2) * smax;
	Ts_a = sqrt(Ts_amax / smax);

	// Step 4: Calculate Ts from jerk constraint
	Ts_j = sqrt(Ts_jmax / smax);


#ifdef __debug
	SerialUSB.println("**********************");
	SerialUSB.println("CalculateTs variables:");
	SerialUSB.print("Ts_d: ");
	SerialUSB.println(Ts_d, 8);
	SerialUSB.print("Ts_v: ");
	SerialUSB.println(Ts_v, 8);
	SerialUSB.print("Ts_a: ");
	SerialUSB.println(Ts_a, 8);
	SerialUSB.print("Ts_j: ");
	SerialUSB.println(Ts_j, 8);
#endif

}


void S4_CurvePlanner::disp_void(){

	jmax = smax * Ts_d;
	amax = jmax * Ts_d;
	vmax = (2 * amax) * Ts_d;

#ifdef __debug
	SerialUSB.print("dmax: ");
	SerialUSB.println(dmax);
	SerialUSB.print("jmax: ");
	SerialUSB.println(jmax);
	SerialUSB.print("amax: ");
	SerialUSB.println(amax);
	SerialUSB.print("vmax: ");
	SerialUSB.println(vmax);
#endif
}

void S4_CurvePlanner::disp_void_rampToCero(){

	jmax_rampToCero = smax * Ts_d;
	amax_rampToCero = jmax_rampToCero * Ts_d;
	vmax_rampToCero = (2 * amax_rampToCero) * Ts_d;

#ifdef __debug
	SerialUSB.print("dmax: ");
	SerialUSB.println(dmax);
	SerialUSB.print("jmax_rampToCero: ");
	SerialUSB.println(jmax_rampToCero);
	SerialUSB.print("amax_rampToCero: ");
	SerialUSB.println(amax_rampToCero);
	SerialUSB.print("vmax_rampToCero: ");
	SerialUSB.println(vmax_rampToCero);
#endif
}


void S4_CurvePlanner::vel_void(){

	jmax = smax * Ts_v;
	amax = pow(Ts, 2) * smax;
#ifdef __debug
	SerialUSB.print("jmax: ");
	SerialUSB.println(jmax);
	SerialUSB.print("amax: ");
	SerialUSB.println(amax);
	SerialUSB.print("vmax: ");
	SerialUSB.println(vmax);
	SerialUSB.print("smax: ");
	SerialUSB.println(smax);
#endif
}



void S4_CurvePlanner::vel_void_rampToCero(){
	if (Ts_v == Ts_a) {
		Ts_a = 0;    }

	jmax_rampToCero = smax * Ts_v;
	amax_rampToCero = pow(Ts_rampToCero, 2) * smax;
#ifdef __debug
	SerialUSB.print("jmax_rampToCero: ");
	SerialUSB.println(jmax_rampToCero);
	SerialUSB.print("amax_rampToCero: ");
	SerialUSB.println(amax_rampToCero);
	SerialUSB.print("vmax_rampToCero: ");
	SerialUSB.println(vmax_rampToCero);
	SerialUSB.print("smax_rampToCero: ");
	SerialUSB.println(smax);
#endif
}



void S4_CurvePlanner::acel_void(){

	jmax = smax * Ts_v;

#ifdef __debug
	SerialUSB.print("jmax: ");
	SerialUSB.println(jmax);
	SerialUSB.print("amax: ");
	SerialUSB.println(amax);
	SerialUSB.print("vmax: ");
	SerialUSB.println(vmax);
	SerialUSB.print("smax: ");
	SerialUSB.println(smax);
#endif
}


void S4_CurvePlanner::acel_void_rampToCero(){

	jmax_rampToCero = smax * Ts_v;

#ifdef __debug
	SerialUSB.print("jmax_rampToCero: ");
	SerialUSB.println(jmax_rampToCero);
	SerialUSB.print("amax: ");
	SerialUSB.println(amax);
	SerialUSB.print("vmax: ");
	SerialUSB.println(vmax);
	SerialUSB.print("smax: ");
	SerialUSB.println(smax);
#endif
}


void S4_CurvePlanner::jerk_void(){

#ifdef __debug
	SerialUSB.print("jmax: ");
	SerialUSB.println(jmax);
	SerialUSB.print("amax: ");
	SerialUSB.println(amax);
	SerialUSB.print("vmax: ");
	SerialUSB.println(vmax);
	SerialUSB.print("smax: ");
	SerialUSB.println(smax);
#endif
}



float S4_CurvePlanner::CalculateTj(float Ts_jerk, float Tj_vmax, float Tj_jmax, float Tj_amax) {

	// Calculate Tdj based on the displacement constraint (Equation 16)
	float c = pow(Ts_jerk, 3);
	float d = (dmax * c) / (54.0f * Tj_jmax);
	float e = pow(dmax, 2) / (16.0f * pow(Tj_jmax, 2));
	float a = (c / 27.0f) + (dmax / (4.0f * Tj_jmax));
	float b = sqrt(d + e);
	float Tjd = cbrt(a + b) + cbrt(a - b) + ((5.0f * Ts_jerk) / 3.0f);

	// Calculate Tvj based on the velocity constraint (Equation 18)
	float Tjv = sqrt((pow(Ts_jerk, 2) / 4) + (Tj_vmax / Tj_jmax)) - ((3 * Ts_jerk) / 2);

	// Calculate Taj based on the acceleration constraint (Equation 20)
	float Tja = (Tj_amax / Tj_jmax) - Ts_jerk;

#ifdef __debug
	SerialUSB.println("*******************************");
	SerialUSB.println("CalculateTj variables: ");
	SerialUSB.print("Tjd: ");
	SerialUSB.println(Tjd);
	SerialUSB.print("Tjv: ");
	SerialUSB.println(Tjv);
	SerialUSB.print("Tja: ");
	SerialUSB.println(Tja);
#endif

	float Tj_ = min3f(Tjd, Tjv, Tja);

	return Tj_;
}


void S4_CurvePlanner::Tjd_void() {
	// Case 1: Only varying jerk and constant jerk
	amax = jmax * (Ts + Tjd);
	vmax = amax * ((2 * Ts) + Tj);

#ifdef __debug
	SerialUSB.println("Case 1: Varying jerk and constant jerk");
	SerialUSB.print("Acceleration: ");
	SerialUSB.println(amax);
	SerialUSB.print("Velocity: ");
	SerialUSB.println(vmax);
#endif
} 

void S4_CurvePlanner::Tjv_void(){

	// Case 2: Maximum velocity reached without max acceleration
	amax = jmax * (Ts + Tjv);

#ifdef __debug
	SerialUSB.println("Case 2: Maximum velocity reached");
	SerialUSB.print("Acceleration: ");
	SerialUSB.println(amax);
	SerialUSB.print("dmax: ");
	SerialUSB.println(dmax);
	SerialUSB.print("jmax: ");
	SerialUSB.println(jmax);
	SerialUSB.print("vmax: ");
	SerialUSB.println(vmax);
#endif
}



float S4_CurvePlanner::CalculateTa(float Ts_, float Tj_, float amax_, float vmax_) {

	//amax = (smax * Ts) * (Ts + Tj);
	float pre_calc = (2 * Ts_) + Tj_;
	// Calculate Tda based on the displacement constraint (Equation 23)
	Tad = ((3 * Tj_) / 2) - (3 * Ts_) + sqrt((pow(pre_calc, 2) / 4 ) + ((qe - qs) / amax_));

	//amax = abs(dmax) / (8 * pow(Ta, 2) + ((3 * Ta) * Tj) + ((6 * Ts) * Ta) + (8 * pow(Ts, 2)) + (2 * pow(Tj, 2)) + ((8 * Ts) * Tj));
	// Calculate Tva based on the velocity constraint (Equation 25)
	Tav = (vmax_ / amax_) - Tj_ + (2 * Ts_);

	// Choose the minimum Ta among the constraints
	// Tad = abs(Tad);
	float Ta_ = fminf(Tad, Tav);

#ifdef __debug
	SerialUSB.println("*******************************");
	SerialUSB.println("CalculateTa variables: ");
	SerialUSB.print("Tad: ");
	SerialUSB.println(Tad);
	SerialUSB.print("Tav: ");
	SerialUSB.println(Tav);
	SerialUSB.print("amax_rampToCero: ");
	SerialUSB.println(amax_rampToCero);
#endif

	return Ta_;
}


void S4_CurvePlanner::Tad_void(){

	vmax = amax * ((2 * Ts) + Tad);
#ifdef __debug
	SerialUSB.print("CASE 1 :");
	SerialUSB.println("Trajectory segments with a constant velocity do not exist");
	SerialUSB.print("real Vmax:");
	SerialUSB.println(vmax);
#endif
}

void S4_CurvePlanner::Tad_void_rampToCero(){
	vmax_rampToCero = amax * ((2 * Ts_rampToCero) + Tad);
#ifdef __debug
	SerialUSB.print("CASE 1 :");
	SerialUSB.println("Trajectory segments with a constant velocity do not exist");
	SerialUSB.print("real Vmax:");
	SerialUSB.println(vmax);
#endif
}

void S4_CurvePlanner::Tav_void(){

#ifdef __debug
	SerialUSB.print("CASE 2 :");
	SerialUSB.println("both the maximum acceleration and velocity can reach their maximums");
#endif
}


float S4_CurvePlanner::CalculateTv() {
	//vmax = (smax * Ts) * (Ts + Tj) * ((2 * Ts) + Tj + Ta);
	// Calculate Tv based on the velocity constraint (Equation 27)
	Tv = ((qe - qs) / vmax) - ((4 * Ts) + (2 * Tj) + Ta);

#ifdef __debug
	SerialUSB.print("vmax");
	SerialUSB.println(vmax);
	SerialUSB.print("Tv");
	SerialUSB.println(Tv);
#endif

	return Tv;
}


float S4_CurvePlanner::CalculateTv_rampToCero(){
	//vmax = (smax * Ts) * (Ts + Tj) * ((2 * Ts) + Tj + Ta);
	// Calculate Tv based on the velocity constraint (Equation 27)
	Tv_rampToCero = ((qe - qs) / vmax_rampToCero) - ((4 * Ts_rampToCero) + (2 * Tj_rampToCero) + Ta_rampToCero);

#ifdef __debug
	SerialUSB.print("vmax_rampToCero");
	SerialUSB.println(vmax_rampToCero);
	SerialUSB.print("Tv_rampToCero:");
	SerialUSB.println(Tv_rampToCero);
#endif

	return Tv_rampToCero;
}




/**************************************************************************************************************************/
bool S4_CurvePlanner::calculateVariables(float Xf, float Xi, float Vi, float Vmax_, float Amax_, float Jmax_, float Smax_) {


	//End position
	qe = Xf;

	//Start Position
	qs = Xi;

	//Initial velocity
	vs = Vi;

	// Calculate dmax from the displacement constraint
	dmax = abs(qs - qe);

	//Snap max
	smax = Smax_;

	//Acceleration max.
	amax = Amax_;

	amax_rampToCero = Amax_;

	// Jerk max.
	jmax = Jmax_;

	jmax_rampToCero = Jmax_;


	//Velocity max. minus initial velocity, for ramp up calculations. If initial velocity is > 0, calculate seperate ramp-down, within same timeframe
	vmax = Vmax_ - vs;

	//Secundary vmax in case ramp-down is diferent from ramp-up
	vmax_rampToCero = Vmax_;

	// Calculate the time intervals using snap
	//Ts = Jmax_ / Smax_;
	//Tj = Amax_ / Jmax_ - Ts;
	//Ta = Vmax_ / Amax_ - Tj - 2 * Ts;
	//Tv = (qe - qs) / Vmax_ - Ta - 2 * Tj - 4 * Ts;

	//Zero
	Ts = Ts_a = Ts_d = Ts_j = Ts_v = 0.0f;

	double_decel_move = false;
	Vi_is_positive = false;

	if (vmax < 0){
		double_decel_move = true;
		vs = vs - (abs(vmax) * 2);
		vmax = abs(vmax);
	}

	//If Vi (Initial velocity) is more then 0 and we are not in a double-deceleration move.
	if (Vi > 0 && !double_decel_move){Vi_is_positive = true;}
	// Calculate the time intervals
	// NOTE: Follow flow chart
	CalculateTs(vmax, amax, jmax);
	Ts = min4f(Ts_d, Ts_v, Ts_a, Ts_j);
	// Choose the minimum Ts among the constraints

	if (Ts == Ts_d){
		// Move is constrained by distance only.
		disp_void();}

	if (Ts == Ts_v){
		// Move is constrained by velocity
		Ts = abs(Ts);
		vel_void();
		Tv = CalculateTv();
	}

	if (Ts == Ts_a){

		// Move is constrained by acceleration
		acel_void();Ta = CalculateTa(Ts, Tj, amax, vmax);

		if (Ta == Tad){Tad_void();}

		if (Ta == Tav){Tav_void();

		CalculateTv();}

	}

	if (Ts == Ts_j){
		//Move is constrained by jerk
		jerk_void();

		Tj = CalculateTj(Ts, vmax, jmax, amax);
		// Choose the minimum Tj among the constraints
		if (Tj == Tjd){
			Tjd_void();
		}else if(Tj == Tjv){
			Tjv_void();
			CalculateTv();
		}else{
			// Case 3: Calculate other time intervals or motion parameters
#ifdef __debug
			SerialUSB.println("Case 3: Calculate Ta");
#endif
			Ta = CalculateTa(Ts, Tj, amax, vmax);
		}
	}

#ifdef __debug
	SerialUSB.println("****************************");
	SerialUSB.println("S-curve Time-Segment Values:");
	SerialUSB.print("Ts: ");
	SerialUSB.println(Ts);
	SerialUSB.print("Tj: ");
	SerialUSB.println(Tj);
	SerialUSB.print("Ta: ");
	SerialUSB.println(Ta);
	SerialUSB.print("Tv: ");
	SerialUSB.println(Tv);
#endif


	//Zorro
	Ts_a = Ts_d = Ts_j = Ts_v = 0;
	//If the initial velocity is higher then (new)max velocity. Ramp down to max velocity and then ramp down to 0.
	//Note: This is also essential if initial velocity is larger then 0.
	//TODO: Handle cases where there are moves in buffer.
	if (double_decel_move || Vi_is_positive){

		CalculateTs(Vmax_, amax_rampToCero, jmax_rampToCero);
		Ts_rampToCero = fminf(Ts_v, Ts_a);

		if (Ts_rampToCero == Ts_d){
			// Move is constrained by distance only.
			disp_void_rampToCero();}

		if (Ts_rampToCero == Ts_v){
			// Move is constrained by velocity
			vel_void_rampToCero();
			Tv_rampToCero = CalculateTv_rampToCero();
		}

		if (Ts_rampToCero == Ts_a){
			// Move is constrained by acceleration
			acel_void();
			Ta_rampToCero = CalculateTa(Ts_rampToCero, Tj_rampToCero, amax_rampToCero, vmax_rampToCero);

			if (Ts_rampToCero == Tad){Tad_void_rampToCero();}

			if (Ts_rampToCero == Tav){Tav_void();CalculateTv_rampToCero();}
		}

		if (Ts_rampToCero == Ts_j){
			//Move is constrained by jerk
			jerk_void();

			Tj_rampToCero = CalculateTj(Ts_rampToCero, vmax_rampToCero, jmax_rampToCero, amax_rampToCero);
			// Choose the minimum Tj among the constraints
			if (Tj == Tjd){
				Tjd_void();
			}else if(Tj == Tjv){
				Tjv_void();
				CalculateTv_rampToCero();
			}else{
				// Case 3: Calculate other time intervals or motion parameters
#ifdef __debug
				SerialUSB.println("Case 3: Calculate Ta");
#endif
				Ta = CalculateTa(Ts_rampToCero, Tj_rampToCero, amax_rampToCero, vmax_rampToCero);
			}
		}
	}

#ifdef __debug
	SerialUSB.println("****************************");
	SerialUSB.println("S-curve Time-Segment Values for rampToCero:");
	SerialUSB.print("Ts_rampToCero: ");
	SerialUSB.println(Ts_rampToCero);
	SerialUSB.print("Tj_rampToCero: ");
	SerialUSB.println(Tj_rampToCero);
	SerialUSB.print("Ta_rampToCero: ");
	SerialUSB.println(Ta_rampToCero);
	SerialUSB.print("Tv_rampToCero: ");
	SerialUSB.println(Tv_rampToCero);
#endif


	// Calculate the maximum motion parameters
	//jmax = smax * Ts;
	//amax = smax * Ts * (Ts + Tj);
	//vmax = smax * Ts * (Ts + Tj) * (2 * Ts + Tj + Ta);
	//dmax = smax * Ts * (Ts + Tj) * (2 * Ts + Tj + Ta) * (4 * Ts + 2 * Tj + Ta + Tv);

	// Calculate transition times for acceleration phase, using Ts (varying jerk), Tj (constant jerk), and Tv(constant velocity - aka cruice)
	t1 = Ts;
	t2 = t1 + Tj;
	t3 = t2 + Ts;
	t4 = t3 + Tj;
	t5 = t4 + Ts;
	t6 = t5 + Tj;
	t7 = t6 + Ts;

	//Constant velocity
	if (Tv <= 0){ t8 = t7;
	} else {
		t8 = t7 + Tv;
	}

	// Calculate transition times for deceleration phase (symetrical to acceleration).

	if (double_decel_move || Vi_is_positive){
		t9 = t8 + Ts_rampToCero;
		t10 = t9 + Tj_rampToCero;
		t11 = t10 + Ts_rampToCero;
		t12 = t11 + Tj_rampToCero;
		t13 = t12 + Ts_rampToCero;
		t14 = t13 + Tj_rampToCero;
		t15 = t14 + Ts_rampToCero;
	}else{
		t9 = t8 + Ts;
		t10 = t9 + Tj;
		t11 = t10 + Ts;
		t12 = t11 + Tj;
		t13 = t12 + Ts;
		t14 = t13 + Tj;
		t15 = t14 + Ts;
	}


#ifdef __debug
	SerialUSB.println("*************************");
	SerialUSB.println("S-curve Transition Times:");
	SerialUSB.print("t1: "); SerialUSB.println(t1);
	SerialUSB.print("t2: "); SerialUSB.println(t2);
	SerialUSB.print("t3: "); SerialUSB.println(t3);
	SerialUSB.print("t4: "); SerialUSB.println(t4);
	SerialUSB.print("t5: "); SerialUSB.println(t5);
	SerialUSB.print("t6: "); SerialUSB.println(t6);
	SerialUSB.print("t7: "); SerialUSB.println(t7);
	SerialUSB.print("t8: "); SerialUSB.println(t8);
	SerialUSB.print("t9: "); SerialUSB.println(t9);
	SerialUSB.print("t10: "); SerialUSB.println(t10);
	SerialUSB.print("t11: "); SerialUSB.println(t11);
	SerialUSB.print("t12: "); SerialUSB.println(t12);
	SerialUSB.print("t13: "); SerialUSB.println(t13);
	SerialUSB.print("t14: "); SerialUSB.println(t14);
	SerialUSB.print("t15: "); SerialUSB.println(t15);
#endif

	/*************************************/
	// Asign duration values for T1 to T15
	T1 = Ts;
	T2 = Tj;
	T3 = Ts;
	T4 = Tj;
	T5 = Ts;
	T6 = Tj;
	T7 = Ts;
	T8 = Tv;

	if (double_decel_move || Vi_is_positive){
		T9 = Ts_rampToCero;
		T10 = Tj_rampToCero;
		T11 = Ts_rampToCero;
		T12 = Tj_rampToCero;
		T13 = Ts_rampToCero;
		T14 = Tj_rampToCero;
		T15 = Ts_rampToCero;

#ifdef __debug
SerialUSB.print("T9: "); SerialUSB.println(T9);
SerialUSB.print("T10: "); SerialUSB.println(T10);
SerialUSB.print("T11: "); SerialUSB.println(T11);
SerialUSB.print("T12: "); SerialUSB.println(T12);
SerialUSB.print("T13: "); SerialUSB.println(T13);
SerialUSB.print("T14: "); SerialUSB.println(T14);
SerialUSB.print("T15: "); SerialUSB.println(T15);
#endif
	}else{
		T9 = Ts;
		T10 = Tj;
		T11 = Ts;
		T12 = Tj;
		T13 = Ts;
		T14 = Tj;
		T15 = Ts;
	}

	// Print the time intervals
#ifdef __debug
	SerialUSB.println("********************************");
	SerialUSB.println("Time Duration for each interval: ");
	SerialUSB.print("T1: "); SerialUSB.println(T1);
	SerialUSB.print("T2: "); SerialUSB.println(T2);
	SerialUSB.print("T3: "); SerialUSB.println(T3);
	SerialUSB.print("T4: "); SerialUSB.println(T4);
	SerialUSB.print("T5: "); SerialUSB.println(T5);
	SerialUSB.print("T6: "); SerialUSB.println(T6);
	SerialUSB.print("T7: "); SerialUSB.println(T7);
	SerialUSB.print("T8: "); SerialUSB.println(T8);
	SerialUSB.print("T9: "); SerialUSB.println(T9);
	SerialUSB.print("T10: "); SerialUSB.println(T10);
	SerialUSB.print("T11: "); SerialUSB.println(T11);
	SerialUSB.print("T12: "); SerialUSB.println(T12);
	SerialUSB.print("T13: "); SerialUSB.println(T13);
	SerialUSB.print("T14: "); SerialUSB.println(T14);
	SerialUSB.print("T15: "); SerialUSB.println(T15);
#endif



	//Calculate Notations
	v1 = vs + (jmax / 6.0) * pow(T1, 2);

	v2 = v1 + (jmax / 2.0) * (T2 * (T1 + T2));

	v3 = v2 + (jmax / 6.0) * T3 * ((3 * T1) + (6 * T2) + (2 * T3));

	v4 = v3 + (amax * T4);

	v5 = v4 - (jmax / 6.0) * pow(T5, 2) + (amax * T5);

	v6 = v5 - (jmax / 2.0) * pow(T6, 2) + (amax - (jmax / 2.0) * T5) * T6;

	v7 = v6 - (jmax / 3.0) * pow(T7, 2) + (amax - ((jmax / 2.0) * T5) - (jmax * T6)) * T7;


	// If vs (initial velocity) > 0, calculate ramp down within same jerk/acceleration constrains. This may lead to position overshoot, but is only the outcome of missing commands.
	// In essence this is just a failsafe if the algorithm recieves a new position command with a unrealistic ramp-dowm, while in motion.
	// If Tv (time in cruice) exist, then we can subtract the time difference and hit the same target.
	// If initial velocity is greater then cruice, do a dual-ramp-dowm.


	//If dual-deceleration and if Vi (initial valocity) is positive, calculate new velocities for ramp-dowm
	if (double_decel_move || Vi_is_positive){
		v1_rd = 0.0f + (jmax_rampToCero / 6.0) * pow(T9, 2);
		v2_rd = v1_rd + (jmax_rampToCero / 2.0) * (T10 * (T9 + T10));
		v3_rd = v2_rd + (jmax_rampToCero / 6.0) * T11 * ((3 * T9) + (6 * T10) + (2 * T11));
		v4_rd = v3_rd + (amax_rampToCero * T12);
		v5_rd = v4_rd - (jmax_rampToCero / 6.0) * pow(T13, 2) + (amax_rampToCero * T13);
		v6_rd = v5_rd - (jmax_rampToCero / 2.0) * pow(T14, 2) + (amax_rampToCero - (jmax_rampToCero / 2.0) * T13) * T14;
		v7_rd = v6_rd - (jmax_rampToCero / 3.0) * pow(T15, 2) + (amax_rampToCero - ((jmax_rampToCero / 2.0) * T13) - (jmax_rampToCero * T14)) * T15;

#ifdef __debug
SerialUSB.print("v1_rd: "); SerialUSB.println(v1_rd);
SerialUSB.print("v2_rd: "); SerialUSB.println(v2_rd);
SerialUSB.print("v3_rd: "); SerialUSB.println(v3_rd);
SerialUSB.print("v4_rd: "); SerialUSB.println(v4_rd);
SerialUSB.print("v5_rd: "); SerialUSB.println(v5_rd);
SerialUSB.print("v6_rd: "); SerialUSB.println(v6_rd);
SerialUSB.print("v7_rd: "); SerialUSB.println(v7_rd);
#endif
	}

#ifdef __debug
	SerialUSB.print("vs: "); SerialUSB.println(vs);
	SerialUSB.print("v1: "); SerialUSB.println(v1);
	SerialUSB.print("v2: "); SerialUSB.println(v2);
	SerialUSB.print("v3: "); SerialUSB.println(v3);
	SerialUSB.print("v4: "); SerialUSB.println(v4);
	SerialUSB.print("v5: "); SerialUSB.println(v5);
	SerialUSB.print("v6: "); SerialUSB.println(v6);
	SerialUSB.print("v7: "); SerialUSB.println(v7);
#endif

	//last_time = 0.0f;
	last_velocity = Vi;
	Last_position = Xi;

	Tf = t15 + T15;

	return true;
}


void S4_CurvePlanner::resetTimeVariables() {
	t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = t9 = t10 = t11 = t12 = t13 = t14 = t15 = 0.0f;
}

void S4_CurvePlanner::Initiate_Move(float Pos){

	// set our global of the new position
	Xf_ = Pos;

	// At this point we are starting to move following the trapezoidal profile
	plannerStartingMovementTimeStamp = TIM2->CNT;

	// take the position from SimpleFOC and set it to our start position
	//    Xi_ = motor->shaft_angle;

	// take the velocity from SimpleFOC and set it to our start velocity
	//    Vi_ = motor->shaft_velocity;

	// TODO: If we are being asked to move but are already in motion, we should go with the current velocity, position, and acceleration
	// and keep moving.

	// Now we need to do our calcs before we start moving
	calculateVariables( Xf_,  Xi_,  Vi_,  _Vmax_,  _Amax_,  _Jmax_,  _Smax_);
	//    motor->target = vel_target; // could possibly put this in the method above
	// Tell user how long this move will take
#ifdef __debug
	SerialUSB.println("Starting to move to a new position");
	SerialUSB.print("Time to complete move (secs):");
	SerialUSB.println(Tf);
	// Velocity and Accel of move
	SerialUSB.print("Velocity for move: ");
	SerialUSB.println(_Vmax_);
	SerialUSB.print("Acceleration: ");
	SerialUSB.println(_Amax_);
	SerialUSB.print("Jerk: ");
	SerialUSB.println(_Jmax_);
	SerialUSB.print("Snap: ");
	SerialUSB.println(_Smax_);

#endif
	// set our global bool so the tick method knows we have a move in motion
	isTrajectoryExecuting = true;
}


//float S4_CurvePlanner::findTf(float Ts, float Tj, float Ta, float Tv, float Td) {
//	float Tf = 0.0f;
//
//	// Perform the summation using a loop
//	for (int i = 1; i <= 15; ++i) {
//		Tf += i * Ts + i * Tj + i * Ta + i * Tv;
//	}
//}



void S4_CurvePlanner::RuntimePlanner(float t) {
	//float currentTrajectoryTime = (((TIM2->CNT+500)/1000) - plannerStartingMov

	if (t >= t0 && t < t1) {
		// Code for the [t0, t1] time range


		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.001f;

		tau1 = t - t0;


		if (!double_decel_move){
			jerk_now = jmax / T1 * tau1;
			acel_now = (jmax / (2.0f * T1)) * pow(tau1, 2);
			vel_target = (jmax / (6.0f * T1)) * pow(tau1, 3) + vs;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}else{
			float v7_2 = v7 + vmax;
			jerk_now = - (jmax / T1) * tau1;
			acel_now = - (jmax / (2.0f * T1)) * pow(tau1, 2);
			vel_target = - (jmax / (6.0f * T1)) * pow(tau1, 3) + v7_2;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;

#ifdef __debug
		print_debug_variables();
#endif

	}


	if (t >= t1 && t < t2) {
		// Code for the [t1, t2] time range

		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.001f;

		tau2 = t - t1;

		if (!double_decel_move){
			jerk_now = jmax;
			acel_now = jmax * tau2 + (jmax / 2.0) * T1;
			vel_target = (jmax / 2.0) * pow(tau2, 2) + (jmax / 2.0) * T1 * tau2 + v1;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}else{
			float v6_2 = v6 + vmax;
			jerk_now = -jmax;
			acel_now = -jmax * tau2 - (jmax / 2.0) * T1;
			vel_target = -(jmax / 2.0) * pow(tau2, 2) - (jmax / 2.0) * T1 * tau2 + v6_2;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;

#ifdef __debug
		print_debug_variables();
#endif

	}


	if (t >= t2 && t < t3) {
		// Code for the [t2, t3] time range

		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau3 = t - t2;

		if (!double_decel_move){
			jerk_now = jmax - (jmax / T3) * tau3;
			acel_now = (jmax * tau3) - (jmax / (2 * T3)) * pow(tau3, 2) + (jmax / 2) * (T1 + (2 * T2));
			vel_target = (jmax / 2.0f) * pow(tau3, 2) - (jmax / (6.0f * T3)) * pow(tau3, 3) + (jmax / 2.0f) * (T1 + (2 * T2)) * tau3 + v2;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;

		}else{
			float v5_2 = v5 + vmax;
			jerk_now = - jmax + ((jmax / T3) * tau3);
			acel_now = - (jmax * tau3) + (jmax / (2 * T3)) * pow(tau3, 2) - (jmax / 2) * (T3 + (2 * T4));
			vel_target = - (jmax / 2.0) * pow(tau3, 2) + ((jmax / (6.0f * T3)) * pow(tau3, 3)) - ((jmax / 2.0) * (T3 + (2 * T4))) * tau3 + v5_2;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;

#ifdef __debug
		print_debug_variables();
#endif
	}

	if (t >= t3 && t < t4) {
		// Code for the [t3, t4] time range

		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau4 = t - t3;

		if (!double_decel_move){
			jerk_now = 0.0;
			acel_now = amax;
			vel_target = (amax * tau4) + v3;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}else{

			float v3_3 = v3 + vmax;
			jerk_now = 0.0;
			acel_now = -amax;
			vel_target = -(amax * tau4) + v3_3;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;

#ifdef __debug
		print_debug_variables();
#endif
	}


	if (t >= t4 && t < t5) {
		// Code for the [t4, t5] time range

		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau5 = t - t4;

		if (!double_decel_move){
			jerk_now = - (jmax / T5) * tau5;
			acel_now = - (jmax / (2.0f * T5)) * pow(tau5, 2) + amax;
			vel_target = -jmax / (6.0f * T5) * pow(tau5, 3) + amax * tau5 + v4;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		} else {
			float v3_2 = v3 + vmax;
			jerk_now = jmax / T5 * tau5;
			acel_now =  (jmax / (2.0f * T5)) * pow(tau5, 2) - amax;
			vel_target = jmax / (6.0f * T5) * pow(tau5, 3) - amax * tau5 + v3_2;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}


		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;


#ifdef __debug
		print_debug_variables();
#endif
	}


	if (t >= t5 && t < t6) {
		// Code for the [t5, t6] time range

		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau6 = t - t5;

		if (!double_decel_move){
			jerk_now = -jmax;
			acel_now = -(jmax * tau6) + amax - (jmax / 2.0) * T5;
			float tempt_cal = amax - (jmax / 2.0) * T5;
			vel_target =  - (jmax / 2.0) * pow(tau6, 2) + tempt_cal * tau6 + v5;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		} else {
			float v2_2 = v2 + vmax;
			jerk_now = jmax;
			acel_now = (jmax * tau6) - amax + (jmax / 2.0) * T5;
			float tempt_cal = amax - (jmax / 2.0) * T5;
			vel_target =   (jmax / 2.0) * pow(tau6, 2) - tempt_cal * tau6 + v2_2;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;

#ifdef __debug
		print_debug_variables();
#endif
	}


	if (t >= t6 && t < t7) {
		// Code for the [t6, t7] time range

		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau7 = t - t6;

		if (!double_decel_move){
			jerk_now = - jmax + ((jmax / T7) * tau7);
			acel_now = - (jmax * tau7) + ((jmax / (2.0f * T7)) * pow(tau7, 2)) + amax - ((jmax / 2.0) * T5) - (jmax * T6);
			vel_target = - (jmax / 2.0) * pow(tau7, 2) + (jmax / (6.0f * T7)) * pow(tau7, 3) + (amax - (jmax / 2.0) * T5 - (jmax * T6)) * tau7 + v6;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}else{
			float v1_2 = v1 + vmax;
			jerk_now = jmax - (jmax / T7) * tau7;
			acel_now =  (jmax * tau7) - ((jmax / (2.0f * T7)) * pow(tau7, 2)) - amax + ((jmax / 2.0) * T5) + (jmax * T6);
			vel_target = (jmax / 2.0) * pow(tau7, 2) - (jmax / (6.0f * T7)) * pow(tau7, 3) - (amax - (jmax / 2.0) * T5 - (jmax * T6)) * tau7 + v1_2;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}


		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;

#ifdef __debug
		print_debug_variables();
#endif
	}


	if (t >= t7 && t < t8) {
		// Code for the [t7, t8] time range

		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau8 = t - t7;

		// Use v7 and q7 as needed for further calculations
		// j(t) and a(t) are both 0 within this range
		jerk_now = 0.0f;
		acel_now = 0.0f;
		vel_target = v7;
		pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;

#ifdef __debug
		print_debug_variables();
#endif
	}


	if (t >= t8 && t < t9) {
		// Code for the [t8, t9] time range

		now_ = TIM2->CNT;
		//deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau9 = t - t8;

		if (!double_decel_move && !Vi_is_positive){
			jerk_now = - (jmax / T9) * tau9;
			acel_now = - (jmax / (2.0f * T9)) * pow(tau9, 2);
			vel_target = - (jmax / (6.0f * T9)) * pow(tau9, 3) + v7;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}else{
			//Execute ramp down to 0 velocity
			jerk_now = - (jmax_rampToCero / T9) * tau9;
			acel_now = - (jmax_rampToCero / (2.0f * T9)) * pow(tau9, 2);
			vel_target = - (jmax_rampToCero / (6.0f * T9)) * pow(tau9, 3) + v7_rd;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;


#ifdef __debug
		print_debug_variables();
#endif
	}


	if (t >= t9 && t < t10) {

		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau10 = t - t9;

		if (!double_decel_move && !Vi_is_positive){
			jerk_now = -jmax;
			acel_now = -jmax * tau10 - (jmax / 2.0) * T9;
			vel_target = -(jmax / 2.0) * pow(tau10, 2) - (jmax / 2.0) * T9 * tau10 + v6;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}else{
			jerk_now = -jmax_rampToCero;
			acel_now = -jmax_rampToCero * tau10 - (jmax_rampToCero / 2.0) * T9;
			vel_target = -(jmax_rampToCero / 2.0) * pow(tau10, 2) - (jmax_rampToCero / 2.0) * T9 * tau10 + v6_rd;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;

#ifdef __debug
		print_debug_variables();
#endif
	}


	if (t >= t10 && t < t11) {
		// Code for the [t10, t11] time range
		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau11 = t - t10;

		if (!double_decel_move && !Vi_is_positive){
			jerk_now = - jmax + ((jmax / T11) * tau11);
			acel_now = - (jmax * tau11) + (jmax / (2 * T11)) * pow(tau11, 2) - (jmax / 2) * (T9 + (2 * T10));
			vel_target = - (jmax / 2.0) * pow(tau11, 2) + (jmax / (6.0f * T11)) * pow(tau11, 3) - (jmax / 2.0) * (T9 + (2 * T10)) * tau11 + v5;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;

		}else{
			jerk_now = - jmax_rampToCero + ((jmax_rampToCero / T11) * tau11);
			acel_now = - (jmax_rampToCero * tau11) + (jmax_rampToCero / (2 * T11)) * pow(tau11, 2) - (jmax_rampToCero / 2) * (T9 + (2 * T10));
			vel_target = - (jmax_rampToCero / 2.0) * pow(tau11, 2) + (jmax_rampToCero / (6.0f * T11)) * pow(tau11, 3) - (jmax_rampToCero / 2.0) * (T9 + (2 * T10)) * tau11 + v5_rd;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;


#ifdef __debug
		print_debug_variables();
#endif
	}


	if (t >= t11 && t < t12) {
		// Code for the [t11, t12] time range

		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau12 = t - t12;

		if (!double_decel_move && !Vi_is_positive){
			jerk_now = 0.0;
			acel_now = -amax;
			vel_target = -(amax * tau12) + v3;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}else{
			jerk_now = 0.0;
			acel_now = -amax_rampToCero;
			vel_target = -(amax_rampToCero * tau12) + v3_rd;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;


#ifdef __debug
		print_debug_variables();
#endif
	}


	if (t >= t12 && t < t13) {
		// Code for the [t12, t13] time range

		now_ = TIM2->CNT;
		//deltaTime = now_ - last_time;

		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau13 = t - t12;

		if (!double_decel_move && !Vi_is_positive){
			jerk_now = jmax / T13 * tau13;
			acel_now =  (jmax / (2.0f * T13)) * pow(tau13, 2) - amax;
			vel_target = jmax / (6.0f * T13) * pow(tau13, 3) - amax * tau13 + v3;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}else{
			jerk_now = jmax_rampToCero / T13 * tau13;
			acel_now =  (jmax_rampToCero / (2.0f * T13)) * pow(tau13, 2) - amax_rampToCero;
			vel_target = jmax_rampToCero / (6.0f * T13) * pow(tau13, 3) - amax_rampToCero * tau13 + v3_rd;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;

#ifdef __debug
		print_debug_variables();
#endif
	}


	if (t >= t13 && t < t14) {
		// Code for the [t13, t14] time range

		now_ = TIM2->CNT;
		deltaTime = now_ - last_time;
		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau14 = t - t13;

		if (!double_decel_move && !Vi_is_positive){
			jerk_now = jmax;
			acel_now = (jmax * tau14) - amax + (jmax / 2.0) * T9;
			float tempt_cal = amax - (jmax / 2.0) * T9;
			vel_target =   (jmax / 2.0) * pow(tau14, 2) - tempt_cal * tau14 + v2;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;

		}else{
			jerk_now = jmax_rampToCero;
			acel_now = (jmax_rampToCero * tau14) - amax_rampToCero + (jmax_rampToCero / 2.0) * T9;
			float tempt_cal_rampToCero = amax_rampToCero - (jmax_rampToCero / 2.0) * T9;
			vel_target =   (jmax_rampToCero / 2.0) * pow(tau14, 2) - tempt_cal_rampToCero * tau14 + v2_rd;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;

#ifdef __debug
		print_debug_variables();
#endif
	}


	if (t >= t14 && t < t15) {
		// Code for the [t14, t15] time range


		now_ = TIM2->CNT;
		//deltaTime = now_ - last_time;

		//convert to seconds
		//deltaTime = deltaTime / 1e6f;
		deltaTime = 0.025f;

		tau15 = t - t14;

		if (!double_decel_move && !Vi_is_positive){
			jerk_now = jmax - (jmax / T15) * tau15;
			acel_now =  (jmax * tau15) - ((jmax / (2.0f * T15)) * pow(tau15, 2)) - amax + ((jmax / 2.0) * T9) + (jmax * T10);
			vel_target = (jmax / 2.0) * pow(tau15, 2) - (jmax / (6.0f * T15)) * pow(tau15, 3) - (amax - (jmax / 2.0) * T9 - (jmax * T10)) * tau15 + v1;
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}else{
			jerk_now = jmax_rampToCero - (jmax_rampToCero / T15) * tau15;
			acel_now =  (jmax_rampToCero * tau15) - ((jmax_rampToCero / (2.0f * T15)) * pow(tau15, 2)) - amax_rampToCero + ((jmax_rampToCero / 2.0) * T9) + (jmax_rampToCero * T10);
			pos_target = Last_position + ((last_velocity + vel_target) / 2.0f + 0.5f * (last_acceleratio + acel_now) * deltaTime) * deltaTime;
		}

		last_time = now_;
		last_velocity = vel_target;
		Last_position = pos_target;
		last_acceleratio = acel_now;

#ifdef __debug
		print_debug_variables();
#endif

	}
}

void S4_CurvePlanner::print_debug_variables(){

	//    SerialUSB.print("jerk_now:");
	//    SerialUSB.print(jerk_now, 5);
	//    SerialUSB.print(",");
	//    SerialUSB.print("acel_now:");
	//    SerialUSB.print(acel_now, 5);
	//    SerialUSB.print(",");
	//    SerialUSB.print("vel_target:");
	//    SerialUSB.print(vel_target, 5);
	//    SerialUSB.print(",");
	//    SerialUSB.print("pos_target:");
	//    SerialUSB.print(pos_target, 5);
	//    SerialUSB.println(",");
}



