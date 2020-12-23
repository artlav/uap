//#######################################################################################//           
//UAP module, Made by Artlav in 2011
//Docking autopilot
//#######################################################################################//
class ap_dock:public iautopilot{
public:
 ap_dock(SAL *sal_in,VESSEL *vessel);
 void start();
 void stop();
 void step(double simt,double simdt);
 void init();

 VESSEL *tgt_ves;
 char tgt_nam[255];
 int port,with_port;
 DOCKHANDLE tgt_dock,our_dock;
 
 int stage;
 double dist,set_a,set_ns,distance;
};
//#######################################################################################//
iautopilot *mk_dock(SAL *sal_in,VESSEL *v);
//#######################################################################################//
