#include "rtklib.h"

#include <stdint.h>
#include <string.h>

#define HY_NUM_SAT_MAX                  32

#define HY_MSG_GROUP_PVT          0x01
#define HY_MSG_GROUP_ASC          0x02
#define HY_MSG_GROUP_ACK          0x05

#define HY_MSG_ID_NAK             0x00
#define HY_MSG_ID_ACK             0x01
#define HY_MSG_ID_DOP             0x04
#define HY_MSG_ID_PVT             0x07
#define HY_MSG_ID_SAT             0x35
#define HY_MSG_ID_EOE             0x61
#define HY_MSG_ID_ASC_SUBFRAME    0x13
#define HY_MSG_ID_ASC_MEAS        0x15

#define HY_MAX_SUBFRAME_DATA_WORDS    16

#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))        // '!!' to make sure this returns 0 or 1

typedef struct
{
    uint32_t TOW;
    uint16_t GDOP;      // Not support.
    uint16_t PDOP;
    uint16_t TDOP;      // Not support.
    uint16_t VDOP;
    uint16_t HDOP;
    uint16_t NDOP;      // Not support.
    uint16_t EDOP;      // Not support.
} Hyfix_PVT_DOP_TypeDef;

typedef struct
{
    uint32_t TOW;
    uint16_t Year;
    uint8_t  Month;
    uint8_t  Day;
    uint8_t  Hour;
    uint8_t  Minute;
    uint8_t  Second;
    uint8_t  Valid;
    uint32_t TimeAcc;
    int32_t  Nano;
    uint8_t  FixType;
    uint8_t  Flags;
    uint8_t  Flags2;
    uint8_t  NumSV;
    int32_t  Lon;
    int32_t  Lat;
    int32_t  Height;
    int32_t  H_MSL;
    uint32_t H_Acc;
    uint32_t V_Acc;
    int32_t  VelN;
    int32_t  VelE;
    int32_t  VelD;
    int32_t  GroundSpeed;
    int32_t  HeadMot;
    uint32_t SpeedAcc;
    uint32_t HeadAcc;
    uint16_t PDOP;
    int8_t   LeapS;
    uint8_t  Reserved1[5];
    int32_t  HeadVeh;
    int16_t  MagDec;
    uint16_t MagAcc;
} Hyfix_PVT_PVT_TypeDef;

typedef struct
{
    uint8_t  GNSS_ID;
    uint8_t  SVID;
    uint8_t  CN0;
    int8_t   Elev;
    int16_t  Azim;
    int16_t  PR_Res;
    uint32_t Flags;
} Hyfix_PVT_SatInfo_TypeDef;

typedef struct
{
    uint32_t TOW;
    uint8_t  Version;
    uint8_t  NumSV;
    uint8_t  Reserved1[2];
} Hyfix_PVT_SAT_TypeDef;

typedef struct
{
    uint8_t GNSS_ID;
    uint8_t SVID;
    uint8_t Reserved1;
    uint8_t FreqID;
    uint8_t NumWords;
    uint8_t Reserved2;
    uint8_t Version;
    uint8_t Reserved3;
    uint32_t DataWords[HY_MAX_SUBFRAME_DATA_WORDS];
} Hyfix_ASC_SUBFRAME_TypeDef;

typedef struct
{
    double   PrMes;
    double   CpMes;
    float    DoMes;
    uint8_t  GNSS_ID;
    uint8_t  SVID;
    uint8_t  SignalID;
    uint8_t  FreqID;
    uint16_t LockTime;
    uint8_t  CN0;
    uint8_t  PrStdev;
    uint8_t  CpStdev;
    uint8_t  DoStdev;
    uint8_t  TrkStat;
    uint8_t  Extra;
} Hyfix_ASC_MEAS_Content_TypeDef;

typedef struct
{
    double   RcvTow;
    uint16_t Week;
    int8_t   LeapS;
    uint8_t  NumMeas;
    uint8_t  RecStat;
    uint8_t  Version;
    uint8_t  Reserved1[2];
    Hyfix_ASC_MEAS_Content_TypeDef Meas[HY_NUM_SAT_MAX];
} Hyfix_ASC_MEAS_TypeDef;

#define HYSYNC1    '$'        /* hyfix message sync code 1 */
#define HYSYNC2    'G'        /* hyfix message sync code 2 */
#define HYSYNC3    'B'        /* hyfix message sync code 3 */

#define PREAMB_CNAV 0x8B        /* cnav preamble */

#define FU1         1           /* hyfix message field types */
#define FU2         2
#define FU4         3
#define FI1         4
#define FI2         5
#define FI4         6
#define FR4         7
#define FR8         8
#define FS32        9

#define P2_10       0.0009765625 /* 2^-10 */

#define CPSTD_VALID 5           /* std-dev threshold of carrier-phase valid */

#define ROUND(x)    (int)floor((x)+0.5)

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((signed char *)(p)))
static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}
static int            I4(unsigned char *p) {int            u; memcpy(&u,p,4); return u;}
static float          R4(unsigned char *p) {float          r; memcpy(&r,p,4); return r;}
static double         R8(unsigned char *p) {double         r; memcpy(&r,p,8); return r;}

static double         I8(unsigned char *p) {return I4(p+4)*4294967296.0+U4(p);}

/* set fields (little-endian) ------------------------------------------------*/
static void setU1(unsigned char *p, unsigned char  u) {*p=u;}
static void setU2(unsigned char *p, unsigned short u) {memcpy(p,&u,2);}
static void setU4(unsigned char *p, unsigned int   u) {memcpy(p,&u,4);}
static void setI1(unsigned char *p, signed char    i) {*p=(unsigned char)i;}
static void setI2(unsigned char *p, short          i) {memcpy(p,&i,2);}
static void setI4(unsigned char *p, int            i) {memcpy(p,&i,4);}
static void setR4(unsigned char *p, float          r) {memcpy(p,&r,4);}
static void setR8(unsigned char *p, double         r) {memcpy(p,&r,8);}

/* hyfix gnssid to system (ref [2] 25) -----------------------------------------*/
static int hyfix_sys(int gnssid)
{
    switch (gnssid) {
        case 0: return SYS_GPS;
        case 1: return SYS_SBS;
        case 2: return SYS_GAL;
        case 3: return SYS_CMP;
        case 5: return SYS_QZS;
        case 6: return SYS_GLO;
    }
    return 0;
}
/* 8-bit week -> full week ---------------------------------------------------*/
static void adj_utcweek(gtime_t time, double *utc)
{
    int week;
    
    if (utc[3]>=256.0) return;
    time2gpst(time,&week);
    utc[3]+=week/256*256;
    if      (utc[3]<week-128) utc[3]+=256.0;
    else if (utc[3]>week+128) utc[3]-=256.0;
}
/* save subframe -------------------------------------------------------------*/
static int save_subfrm(int sat, raw_t *raw)
{
    unsigned char *p=raw->buff+6,*q;
    int i,j,n,id=(U4(p+6)>>2)&0x7;
    
    trace(4,"save_subfrm: sat=%2d id=%d\n",sat,id);
    
    if (id<1||5<id) return 0;
    
    q=raw->subfrm[sat-1]+(id-1)*30;
    
    for (i=n=0,p+=2;i<10;i++,p+=4) {
        for (j=23;j>=0;j--) {
            *q=(*q<<1)+((U4(p)>>j)&1); if (++n%8==0) q++;
        }
    }
    return id;
}
/* decode ephemeris ----------------------------------------------------------*/
static int decode_ephem(int sat, raw_t *raw)
{
    eph_t eph={0};
    
    trace(4,"decode_ephem: sat=%2d\n",sat);
    
    if (decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL)!=1||
        decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL,NULL)!=2||
        decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL,NULL)!=3) return 0;
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode&&
            eph.iodc==raw->nav.eph[sat-1].iodc) return 0; /* unchanged */
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    return 2;
}
/* decode almanac and ion/utc ------------------------------------------------*/
static int decode_alm1(int sat, raw_t *raw)
{
    int sys=satsys(sat,NULL);
    
    trace(4,"decode_alm1 : sat=%2d\n",sat);
    
    if (sys==SYS_GPS) {
        decode_frame(raw->subfrm[sat-1]+90,NULL,raw->nav.alm,raw->nav.ion_gps,
                     raw->nav.utc_gps,&raw->nav.leaps);
        adj_utcweek(raw->time,raw->nav.utc_gps);
    }
    else if (sys==SYS_QZS) {
        decode_frame(raw->subfrm[sat-1]+90,NULL,raw->nav.alm,raw->nav.ion_qzs,
                     raw->nav.utc_qzs,&raw->nav.leaps);
        adj_utcweek(raw->time,raw->nav.utc_qzs);
    }
    return 9;
}
/* decode almanac ------------------------------------------------------------*/
static int decode_alm2(int sat, raw_t *raw)
{
    int sys=satsys(sat,NULL);
    
    trace(4,"decode_alm2 : sat=%2d\n",sat);
    
    if (sys==SYS_GPS) {
        decode_frame(raw->subfrm[sat-1]+120,NULL,raw->nav.alm,NULL,NULL,NULL);
    }
    else if (sys==SYS_QZS) {
        decode_frame(raw->subfrm[sat-1]+120,NULL,raw->nav.alm,raw->nav.ion_qzs,
                     raw->nav.utc_qzs,&raw->nav.leaps);
        adj_utcweek(raw->time,raw->nav.utc_qzs);
    }
    return  0;
}
/* decode gps and qzss navigation data ---------------------------------------*/
static int decode_nav(raw_t *raw, int sat, int off)
{
    unsigned int words[10];
    int i,id;
    unsigned char *p=raw->buff+6+off;
    
    if (raw->len<48+off) {
        trace(2,"hyfix rawsfrbx length error: sat=%d len=%d\n",sat,raw->len);
        return -1;
    }
    if ((U4(p)>>24)==PREAMB_CNAV) {
        trace(3,"hyfix rawsfrbx cnav not supported sat=%d prn=%d\n",sat,
              (U4(p)>>18)&0x3F);
        return 0;
    }
    for (i=0;i<10;i++,p+=4) words[i]=U4(p)>>6; /* 24 bits without parity */
    
    id=(words[1]>>2)&7;
    if (id<1||5<id) {
        trace(2,"hyfix rawsfrbx subfrm id error: sat=%2d id=%d len=%d\n",sat,id,
              raw->len);
        return -1;
    }
    for (i=0;i<10;i++) {
        setbitu(raw->subfrm[sat-1]+(id-1)*30,i*24,24,words[i]);
    }
    if (id==3) return decode_ephem(sat,raw);
    if (id==4) return decode_alm1 (sat,raw);
    if (id==5) return decode_alm2 (sat,raw);
    return 0;
}
/* decode galileo navigation data --------------------------------------------*/
static int decode_enav(raw_t *raw, int sat, int off)
{
    eph_t eph={0};
    unsigned char *p=raw->buff+6+off,buff[32],crc_buff[26]={0};
    int i,j,k,part1,page1,part2,page2,type;
    
    if (raw->len<44+off) {
        trace(2,"hyfix rawsfrbx length error: sat=%d len=%d\n",sat,raw->len);
        return -1;
    }
    for (i=k=0;i<8;i++,p+=4) for (j=0;j<4;j++) {
        buff[k++]=p[3-j];
    }
    part1=getbitu(buff   ,0,1);
    page1=getbitu(buff   ,1,1);
    part2=getbitu(buff+16,0,1);
    page2=getbitu(buff+16,1,1);
    
    /* skip alert page */
    if (page1==1||page2==1) return 0;
    
    /* test even-odd parts */
    if (part1!=0||part2!=1) {
        trace(2,"hyfix rawsfrbx gal page even/odd error: sat=%2d\n",sat);
        return -1;
    }
    /* test crc (4(pad) + 114 + 82 bits) */
    for (i=0,j=  4;i<15;i++,j+=8) setbitu(crc_buff,j,8,getbitu(buff   ,i*8,8));
    for (i=0,j=118;i<11;i++,j+=8) setbitu(crc_buff,j,8,getbitu(buff+16,i*8,8));
    if (rtk_crc24q(crc_buff,25)!=getbitu(buff+16,82,24)) {
        trace(2,"hyfix rawsfrbx gal page crc error: sat=%2d\n",sat);
        return -1;
    }
    type=getbitu(buff,2,6); /* word type */
    
    /* skip word except for ephemeris, iono, utc parameters */
    if (type>6) return 0;
    
    /* clear word 0-6 flags */
    if (type==2) raw->subfrm[sat-1][112]=0;
    
    /* save page data (112 + 16 bits) to frame buffer */
    k=type*16;
    for (i=0,j=2;i<14;i++,j+=8) raw->subfrm[sat-1][k++]=getbitu(buff   ,j,8);
    for (i=0,j=2;i< 2;i++,j+=8) raw->subfrm[sat-1][k++]=getbitu(buff+16,j,8);
    
    /* test word 0-6 flags */
    //raw->subfrm[sat-1][112]|=(1<<type);
    //if (raw->subfrm[sat-1][112]!=0x7F) return 0;
    
    if (strstr(raw->opt,"-GALFNAV")) {
        return 0;
    }
    /* decode galileo inav ephemeris */
    if (!decode_gal_inav(raw->subfrm[sat-1],&eph)) {
        return 0;
    }
    /* test svid consistency */
    if (eph.sat!=sat) {
        trace(2,"hyfix rawsfrbx gal svid error: sat=%2d %2d\n",sat,eph.sat);
        return -1;
    }
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode&& /* unchanged */
            timediff(eph.toe,raw->nav.eph[sat-1].toe)==0.0&&
            timediff(eph.toc,raw->nav.eph[sat-1].toc)==0.0) return 0;
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    return 2;
}
/* decode beidou navigation data ---------------------------------------------*/
static int decode_cnav(raw_t *raw, int sat, int off)
{
    eph_t eph={0};
    unsigned int words[10];
    int i,id,pgn,prn;
    unsigned char *p=raw->buff+6+off;
    
    if (raw->len<48+off) {
        trace(2,"hyfix rawsfrbx length error: sat=%d len=%d\n",sat,raw->len);
        return -1;
    }
    for (i=0;i<10;i++,p+=4) words[i]=U4(p)&0x3FFFFFFF; /* 30 bits */
    
    satsys(sat,&prn);
    id=(words[0]>>12)&0x07; /* subframe id (3bit) */
    if (id<1||5<id) {
        trace(2,"hyfix rawsfrbx subfrm id error: sat=%2d\n",sat);
        return -1;
    }
    if (prn>5&&prn<59) { /* IGSO/MEO */
        
        for (i=0;i<10;i++) {
            setbitu(raw->subfrm[sat-1]+(id-1)*38,i*30,30,words[i]);
        }
        if (id!=3) return 0;
        
        /* decode beidou D1 ephemeris */
        if (!decode_bds_d1(raw->subfrm[sat-1],&eph)) return 0;
    }
    else { /* GEO (C01-05, C59-63) */
        if (id!=1) return 0;
        
        /* subframe 1 */
        pgn=(words[1]>>14)&0x0F; /* page number (4bit) */
        if (pgn<1||10<pgn) {
            trace(2,"hyfix rawsfrbx page number error: sat=%2d\n",sat);
            return -1;
        }
        for (i=0;i<10;i++) {
            setbitu(raw->subfrm[sat-1]+(pgn-1)*38,i*30,30,words[i]);
        }
        if (pgn!=10) return 0;
        
        /* decode beidou D2 ephemeris */
        if (!decode_bds_d2(raw->subfrm[sat-1],&eph)) return 0;
    }
    if (!strstr(raw->opt,"-EPHALL")) {
        if (timediff(eph.toe,raw->nav.eph[sat-1].toe)==0.0&&
            eph.iode==raw->nav.eph[sat-1].iode&&
            eph.iodc==raw->nav.eph[sat-1].iodc) return 0; /* unchanged */
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    return 2;
}
/* decode glonass navigation data --------------------------------------------*/
static int decode_gnav(raw_t *raw, int sat, int off, int frq)
{
    geph_t geph={0};
    int i,j,k,m,prn;
    unsigned char *p=raw->buff+6+off,buff[64],*fid;
    
    satsys(sat,&prn);
    
    if (raw->len<24+off) {
        trace(2,"hyfix rawsfrbx gnav length error: len=%d\n",raw->len);
        return -1;
    }
    for (i=k=0;i<4;i++,p+=4) for (j=0;j<4;j++) {
        buff[k++]=p[3-j];
    }
    /* test hamming of glonass string */
    if (!test_glostr(buff)) {
        trace(2,"hyfix rawsfrbx glo string hamming error: sat=%2d\n",sat);
        return -1;
    }
    m=getbitu(buff,1,4);
    if (m<1||15<m) {
        trace(2,"hyfix rawsfrbx glo string no error: sat=%2d\n",sat);
        return -1;
    }
    /* flush frame buffer if frame-id changed */
    fid=raw->subfrm[sat-1]+150;
    if (fid[0]!=buff[12]||fid[1]!=buff[13]) {
        for (i=0;i<4;i++) memset(raw->subfrm[sat-1]+i*10,0,10);
        memcpy(fid,buff+12,2); /* save frame-id */
    }
    memcpy(raw->subfrm[sat-1]+(m-1)*10,buff,10);
    
    if (m!=4) return 0;
    
    /* decode glonass ephemeris strings */
    geph.tof=raw->time;
    if (!decode_glostr(raw->subfrm[sat-1],&geph)||geph.sat!=sat) return 0;
    geph.frq=frq-7;
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (geph.iode==raw->nav.geph[prn-1].iode) return 0; /* unchanged */
    }
    raw->nav.geph[prn-1]=geph;
    raw->ephsat=sat;
    return 2;
}
/* decode sbas navigation data -----------------------------------------------*/
static int decode_snav(raw_t *raw, int sat, int off)
{
    int i,j,k,prn,tow,week;
    unsigned char *p=raw->buff+6+off,buff[64];
    
    if (raw->len<40+off) {
        trace(2,"hyfix rawsfrbx snav length error: len=%d\n",raw->len);
        return -1;
    }
    tow=(int)time2gpst(timeadd(raw->time,-1.0),&week);
    satsys(sat,&prn);
    raw->sbsmsg.prn=prn;
    raw->sbsmsg.tow=tow;
    raw->sbsmsg.week=week;
    for (i=k=0;i<8;i++,p+=4) for (j=0;j<4;j++) {
        buff[k++]=p[3-j];
    }
    memcpy(raw->sbsmsg.msg,buff,29);
    raw->sbsmsg.msg[28]&=0xC0;
    return 3;
}
/* decode hyfix-rxm-sfrbx: raw subframe data (ref [3][4][5]) -------------------*/
static int decode_rxmsfrbx(raw_t *raw)
{
    int prn,sat,sys;
    unsigned char *p=raw->buff+6;
    
    trace(4,"decode_rxmsfrbx: len=%d\n",raw->len);
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"HY RXM-SFRBX (%4d): sys=%d prn=%3d",raw->len,
                U1(p),U1(p+1));
    }
    if (!(sys=hyfix_sys(U1(p)))) {
        trace(2,"hyfix rxmsfrbx sys id error: sys=%d\n",U1(p));
        return -1;
    }
    prn=U1(p+1)+(sys==SYS_QZS?192:0);
    if (!(sat=satno(sys,prn))) {
        if (sys==SYS_GLO&&prn==255) {
            return 0; /* suppress error for unknown glo satellite */
        }
        trace(2,"hyfix rxmsfrbx sat number error: sys=%d prn=%d\n",sys,prn);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: return decode_nav (raw,sat,8);
        case SYS_QZS: return decode_nav (raw,sat,8);
        case SYS_GAL: return decode_enav(raw,sat,8);
        case SYS_CMP: return decode_cnav(raw,sat,8);
        case SYS_GLO: return decode_gnav(raw,sat,8,U1(p+3));
        case SYS_SBS: return decode_snav(raw,sat,8);
    }
    return 0;
}


#define MAXFIELD 100

static int parse_fields(char* const buffer, char** val)
{
    char* p, * q;
    int n = 0;

    /* parse fields */
    for (p = buffer; *p && n < MAXFIELD; p = q + 1) {
        if (q = strchr(p, ',')) {
            val[n++] = p; *q = '\0';
        }
        else break;
    }
    if (p != NULL)
    {
        if (q = strchr(p, '\n'))
            *q = '\0';
        val[n++] = p;
    }
    return n;
}

/* decode hyfix raw message --------------------------------------------------*/
static int decode_hyfix(raw_t *raw, int type)
{
	unsigned char* tx_buff = raw->buff;
	int i = 0;
    double dt=0.0,blh[3]={0};
    if (type == 1000)
    {
        /* eph */
        Hyfix_ASC_SUBFRAME_TypeDef* ASC_SubFrame = (Hyfix_ASC_SUBFRAME_TypeDef*)tx_buff;

        raw->len = sizeof(Hyfix_ASC_SUBFRAME_TypeDef);

        return decode_rxmsfrbx(raw);
    }
    else if (type == 2000)
    {
        Hyfix_ASC_MEAS_TypeDef* ASC_Meas = (Hyfix_ASC_MEAS_TypeDef*)tx_buff;
        double ws = ASC_Meas->RcvTow;
        int wn = ASC_Meas->Week;
        gtime_t time = gpst2time(wn, ws);
        for (i = 0; i < ASC_Meas->NumMeas; i++)
        {
            Hyfix_ASC_MEAS_Content_TypeDef *obs = ASC_Meas->Meas + i;
            int f = 0;
            int sys = 0;
            int prn = obs->SVID;
            int sat = 0;
            int j =0;
            int code = 0;
            int trstat = obs->TrkStat;

            int prValid = BIT_CHECK(trstat, 0);
            int cpValid = BIT_CHECK(trstat, 1); /* false => cycle slip */
            int halfCycle = BIT_CHECK(trstat, 2);
            int subHalfCycle = BIT_CHECK(trstat, 3);

            if (obs->GNSS_ID == 0)
            {
                /* GPS L1C/A (0) L5Q (7) */
                sys = SYS_GPS;
                if (obs->SignalID==0)
                {
                    f = 0;
                    code = CODE_L1C;
                }
                else if (obs->SignalID==7)
                {
                    f = 2;
                    code = CODE_L5Q;
                }
                else continue;
            }
            else if (obs->GNSS_ID == 2)
            {
                /* GAL E1C (0) E5aQ (4) */
                sys = SYS_GAL;
                if (obs->SignalID == 0)
                {
                    f = 0;
                    code = CODE_L1C; 
                }
                else if (obs->SignalID == 4)
                {
                    f = 2;
                    code = CODE_L5Q;
                }
                else continue;
            }
            else if (obs->GNSS_ID == 3)
            {
                /* BDS B1D1 */
                sys = SYS_CMP;
                if (obs->SignalID == 0)
                {
                    f = 0;
                    code = CODE_L1I;
                }
                else continue;
            }
            else if (obs->GNSS_ID == 5)
            {
                /* QZS L1C/A L5Q */
                sys = SYS_QZS;
                if (obs->SignalID == 0)
                {
                    f = 0;
                    code = CODE_L1C;
                }
                else if (obs->SignalID == 9) /* L5Q */
                {
                    f = 2;
                    code = CODE_L5Q;
                }
                else continue;
            }
            else if (obs->GNSS_ID == 6)
            {
                /* GLO L1OF */
                sys = SYS_GLO;
                if (obs->SignalID == 0)
                {
                    f = 0;
                    code = CODE_L1C;
                }
                else continue;
            }
            else continue;
            if (!(sat=satno(sys, prn))) continue;
            for (j=0;j< raw->obs.n;++j)
            {
                if (raw->obs.data[j].sat==sat && raw->obs.data[j].rcv==0) break;
            }
            if (j==raw->obs.n)
            {
                /* new satellite */
                /* find the best location */
                for (j = 0; j < raw->obs.n; ++j)
                {
                    if (raw->obs.data[j].sat==0) break;
                }
                if (j==raw->obs.n)
                {
                    if (raw->obs.n==MAXOBS) continue;
                    memset(raw->obs.data+j, 0, sizeof(obsd_t));
                    ++raw->obs.n;
                }
            }
            raw->obs.data[j].sat = sat;
            raw->obs.data[j].code[f] = code;
			raw->obs.data[j].P[f] = prValid?(obs->PrMes):(0.0);
			raw->obs.data[j].L[f] = obs->CpMes;
			raw->obs.data[j].D[f] = obs->DoMes;
			raw->obs.data[j].SNR[f] = obs->CN0*4.0;
            raw->obs.data[j].LLI[f] = cpValid?(0):(1);
            raw->obs.data[j].time = time;
            raw->obs.data[j].rcv = 0;
        }
        for (i=0;i<raw->obs.n;++i)
        {
            if (raw->obs.data[i].rcv>0) continue;
			dt = timediff(raw->obs.data[i].time, time);
            if (fabs(dt)>1.5)
                memset(raw->obs.data+i, 0, sizeof(obsd_t));
        }
		if (raw->outtype)
			sprintf(raw->msgtype,"ASC_MEAS: %f, %u, %d, %u, %u\r\n",
            ASC_Meas->RcvTow,
            ASC_Meas->Week,
            ASC_Meas->LeapS,
            ASC_Meas->NumMeas,
            ASC_Meas->RecStat
        );
        return 1; /* raw obs */
    }
    else if (type == 3000)
    {
		Hyfix_PVT_PVT_TypeDef* PVT_Info = (Hyfix_PVT_PVT_TypeDef*)tx_buff;

		if (PVT_Info->Valid==23) {
			blh[0] = PVT_Info->Lat * 1e-7 * D2R;
			blh[1] = PVT_Info->Lon * 1e-7 * D2R;
			blh[2] = PVT_Info->H_MSL * 1e-3;
			pos2ecef(blh, raw->sta.pos);
			if (raw->outtype) sprintf(raw->msgtype,"NAV-PVT: %10.3f,%14.9f,%14.9f,%10.4f,%14.4f,%14.4f,%14.4f\r\n",
				PVT_Info->TOW/1000.0, blh[0],blh[1],blh[2],raw->sta.pos[0],raw->sta.pos[1],raw->sta.pos[2]);

		}
		#if 0
		if (raw->outtype) sprintf(raw->msgtype,"NAV-PVT: Tow=%u, DateTime=%.4u-%.2u-%.2u %.2u:%.2u:%.2u.%d, FixType=%u, NumSv=%u, LLA=(%f, %f, %f), DoP=%f, leapS:%d, Speed=%f, Heading=%f, Valid=%u \r\n",
			PVT_Info->TOW,
			PVT_Info->Year,
			PVT_Info->Month,
			PVT_Info->Day,
			PVT_Info->Hour,
			PVT_Info->Minute,
			PVT_Info->Second,
			PVT_Info->Nano,
			PVT_Info->FixType,
			PVT_Info->NumSV,
			PVT_Info->Lat * 1e-7,
			PVT_Info->Lon * 1e-7,
			PVT_Info->H_MSL * 1e-3,
			PVT_Info->PDOP * 1e-2,
			PVT_Info->LeapS,
			PVT_Info->GroundSpeed * 1e-3,
			PVT_Info->HeadMot * 1e-5,
			PVT_Info->Valid);
        #endif
        return 5; /* PVT */
    }
    return 0;
}
/* sync code -----------------------------------------------------------------*/
static int sync_hyfix(unsigned char *buff, unsigned char data)
{
    /* $GBROV $GBREF */
    buff[0] = buff[1]; buff[1]=buff[2]; buff[2] = buff[3]; buff[3] = buff[4]; buff[4] = buff[5]; buff[5]=data;
    return buff[0]==HYSYNC1&&buff[1]==HYSYNC2 && buff[2] == HYSYNC3 && buff[3] == 'R' && ( (buff[4] == 'O' && buff[5] == 'V') || (buff[4] == 'E' && buff[5] == 'F'));
}
/* input hyfix raw message from stream */
extern int input_hyfix(raw_t *raw, unsigned char data)
{
	int nseg=0,nloc[4]={0},i=0,len_t=0,len_l=0,len=0,exp_nbyte=0,offset=0,type=0;
	char temp[30]={0};
	trace(5,"input_hyfix: data=%02x\n",data);

	/* synchronize frame */

	if (raw->nbyte >= MAXRAWLEN)
		raw->nbyte = 0;
	if (raw->nbyte==0) {
		memset(raw->buff, 0, sizeof(raw->buff));
		if (data!='$') return 0;
		raw->buff[raw->nbyte++] = data;
		return 0;
	}
	raw->buff[raw->nbyte++]=data;
	if (raw->nbyte<6) return 0;

	if (raw->buff[0]=='$'&& raw->buff[1] == 'G' && raw->buff[2] == 'B' && raw->buff[3] == 'R' && ( (raw->buff[4] == 'O'&& raw->buff[5] == 'V')||(raw->buff[4] == 'E' && raw->buff[5] == 'F')))
	{
		nseg = 0;
		memset(nloc,0,sizeof(nloc));
		for (i = 0; i < raw->nbyte; ++i)
		{
			if (raw->buff[i] == ',') nloc[nseg++] = i;
			if (nseg==3) break;
		}
		if (nseg < 3) return 0;

		len_t = nloc[1]-nloc[0]-1;
		len_l = nloc[2]-nloc[1]-1;
		if (len_t >10|| len_l>10)
        {
            raw->nbyte = 0;
            return 0;
        }
        memcpy(temp, raw->buff+nloc[0]+1, sizeof(char)*len_t);
		type = atoi(temp);
        memcpy(temp, raw->buff+nloc[1]+1, sizeof(char)*len_l);
		len = atoi(temp);

		exp_nbyte = len + nloc[2] + 1;

        if (raw->nbyte< exp_nbyte)
            return 0;

        if (raw->buff[4] == 'O' && raw->buff[5] == 'V')
        {
			/* decode hyfix raw message */
			if (raw->outtype) sprintf(raw->msgtype,"hyfix rov=%4i,%4i",type,raw->nbyte);
			offset = nloc[2] + 1;
			if (type==1000) offset -= 6;
			for (i=0;i< len;++i)
				raw->buff[i] = raw->buff[i+offset];
			raw->nbyte = 0;
			return decode_hyfix(raw, type);
		}
		else
		{
			if (raw->outtype) sprintf(raw->msgtype,"hyfix ref=%4i,%4i",type,raw->nbyte);
			offset = nloc[2] + 1;
			for (i = 0; i < len; ++i)
				raw->buff[i] = raw->buff[i + offset];
			raw->nbyte = len;
            return 10; /* base station data */
        }
    }

    if (data == '\r' || data == '\n')
    {
        raw->nbyte = 0;
    }
    return 0;
}
/* input hyfix raw message from file -------------------------------------------
* fetch next hyfix raw data and input a message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_hyfixf(raw_t *raw, FILE *fp)
{
    int i,data,ret=-2;
    
    trace(4,"input_hyfixf:\n");

    while (fp != NULL && !feof(fp))
    {
        if ((data = fgetc(fp)) == EOF)
            break;
        if ((ret = input_hyfix(raw, data))) break;
    }
    return ret;
}

