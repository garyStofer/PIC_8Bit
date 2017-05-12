extern void InitUART( void );
extern void TxChar(char ch);
extern void TxText(const  char *pch);
extern void ShowPrompt(void);
extern void TxValU(uns8 v);
extern void TxNibble(uns8 v);
extern void TxValH(uns8 v);
extern void TxValH16(uns16 v);
extern void TxValS(int8 v);
extern void TxNextLine(void);
extern char RxChar(void);
extern unsigned char  RxNumU(void);
extern signed char RxNumS(void);

#define _B38400		(_ClkOut*100000/(4*38400) - 1) 


