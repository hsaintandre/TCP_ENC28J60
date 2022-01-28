/*Opcodes*/
#define RCR         0x00
#define RBM         0x3A
#define WCR         0x40
#define WBM         0x7A
#define BFS         0x80
#define BFC         0xA0
#define reset       0xFF

/*Common registers*/
#define ECON1       0x1F
#define ECON2       0x1E
#define ESTAT       0x1D
#define EIR         0x1C
#define EIE         0x1B

/*Bank 0*/
#define ERDPTL      0x00
#define ERDPTH      0x01
#define EWRPTL      0x02
#define EWRPTH      0x03
#define ETXSTL      0x04
#define ETXSTH      0x05
#define ETXNDL      0x06
#define ETXNDH      0x07
#define ERXSTL      0x08
#define ERXSTH      0x09
#define ERXNDL      0x0A
#define ERXNDH      0x0B
#define ERXRDPTL    0x0C
#define ERXRDPTH    0x0D
#define ERXWRPTL    0x0E
#define ERXWRPTH    0x0F

/*Bank 1*/
#define ERXFCON     0x18
#define EPKTCNT     0x19

/*Bank 2*/
#define MACON1      0x00
#define MACON3      0x02
#define MACON4      0x03
#define MAIPGL      0x06
#define MAIPGH      0x07
#define MAMXFLL     0x0A
#define MAMXFLH     0x0B
#define MICMD       0x12
#define MIREGADR    0x14
#define MIWRL       0x16
#define MIWRH       0x17

/*Bank 3*/
#define MAADR5      0x00
#define MAADR6      0x01
#define MAADR3      0x02
#define MAADR4      0x03
#define MAADR1      0x04
#define MAADR2      0x05