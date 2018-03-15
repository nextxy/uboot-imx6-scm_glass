#ifndef CONFIG_GUI_H
#define CONFIG_GUI_H



void LCD_Draw_Circle1(u16 x0,u16 y0,int r,u16 color);
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);            //hua juxing   kongxin
void Show_Font(u16 x,u16 y,u8 size,u8 mode);         //xie  hanzi
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color);     //hua  juxing   shixin
 void LCD_Straight_Line(u16 x1, u16 y1,u16 size);//hua shizi
void level1(u16 x0, u16 y0, u16 dx, u16 dy);    //к╝ф╫рг
void level2(u16 yx,u16 yy);

void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);//  hua   zhixian

void LCD_GBK(u16 x,u16 y,int qh,int ql,u8 mode);
//void video__logo(void);
#endif
