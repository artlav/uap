//#######################################################################################//
//UAP module, Made by Artlav in 2011
//Aerial autopilots
//#######################################################################################//
class ap_runway_off:public iautopilot{
public:
 ap_runway_off(SAL *sal_in,VESSEL *vessel);
 void start();
 void stop();
 void step(double simt,double simdt);
 void init();
 void reset();

 double altitude,heading,v1,thrust;
 int use_target,use_grp,stage,gear_key;
 char target[255];
 OBJHANDLE tgt_obj;
};
//#######################################################################################//
class ap_air_hold:public iautopilot{
public:
 ap_air_hold(SAL *sal_in,VESSEL *vessel);
 void start();
 void stop();
 void step(double simt,double simdt);
 void init();

 double altitude,heading,velocity,alt_rate,hdg_rate,vel_rate;
 int use_grp,onoff;
};
//#######################################################################################//
iautopilot *mk_runway_off(SAL *sal_in,VESSEL *v);
iautopilot *mk_air_hold(SAL *sal_in,VESSEL *v);
//#######################################################################################//
