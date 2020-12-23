//#######################################################################################//
//UAP planner MFD, Made by Artlav in 2011
//I don't remember WTF that was supposed to be
//#######################################################################################//
#define pages 3
//#######################################################################################//
class uap_planner_mfd:public MFD{
public:
 uap_planner_mfd(DWORD w,DWORD h,VESSEL *vessel);
 ~uap_planner_mfd();
 char *ButtonLabel(int bt);
 int ButtonMenu(const MFDBUTTONMENU **menu)const;
 bool ConsumeButton(int bt,int event);
 bool ConsumeKeyBuffered(DWORD key);
 void Update(HDC hDC);
 static int MsgProc(UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam);
 void get_tgt();
 void do_enter(int n);
 bool do_set(char *nam);
 
 int mfd_seq,scrx,scry;
};
//#######################################################################################//

