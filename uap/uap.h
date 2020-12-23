//#######################################################################################//
//UAP MFD, Made by Artlav in 2011
//The UI
//#######################################################################################//
#define pages 3
//#######################################################################################//
class uapmfd:public MFD{
public:
 uapmfd(DWORD w,DWORD h,VESSEL *vessel);
 ~uapmfd();
 char *ButtonLabel(int bt);
 int ButtonMenu(const MFDBUTTONMENU **menu)const;
 bool ConsumeButton(int bt,int event);
 bool ConsumeKeyBuffered(DWORD key);
 bool ConsumeKeyImmediate(char *kstate);
 void Update(HDC hDC);
 static int MsgProc(UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam);
 void get_tgt();
 void do_enter();
 bool set_cur(char *in);
 void seq_op(int op,int tp);
 void write_vars(HDC hDC,int &line,iautopilot *ap,int use_sel);
 void write_seqs(HDC hDC,int &line,int n,int use_sel);
 void write_avls(HDC hDC,int &line,int use_sel);
 void write_error(HDC hDC,int &line);
 
 int mfd_seq,scrx,scry,page,sel,sels,can_enter,ap_n;
};
//#######################################################################################//
int uap_mfdmode,uap_planner_mfdmode;
//#######################################################################################//

