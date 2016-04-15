#include <usbhub.h>
#include <FX1S.h>
//by program-plc.blogspot.com
#define FX1Sbaud 19200
#define FX1Sformat SERIAL_8N1
#define FX1Stimeout 100


enum
{
  FX1SPACKET1,
  FX1STOTAL_NO_OF_PACKETS
};
FX1SPacket FX1Spackets[FX1STOTAL_NO_OF_PACKETS];

FX1SpacketPointer FX1Spacket1 = &FX1Spackets[FX1SPACKET1];


#define NFC_Hardware "ACR122U"
#define NFC_Tags_Max 7

unsigned int NFC_FX1SwriteD[NFC_Tags_Max+1];


USB Usb;
USB_DEVICE_DESCRIPTOR buf;

uint8_t NFC_USB_Address;
uint8_t NFC_Code_Return;
uint8_t NFC_USB_State;
bool NFC_USB_RUN = false;
uint8_t  NFC_Receive[64];

String NFC_Receive_String = "";
unsigned long NFC_Get_UID_Timeout;
uint16_t NFC_Receive_Total_Data;
uint16_t NFC_Receive_Size;
uint16_t NFC_Tags_Size;
uint8_t  NFC_Tags[NFC_Tags_Max];
unsigned int NFC_Send_Success;
  
uint8_t NFC_USB_Order;
uint8_t NFC_EPP_OUT = 0x22; //NFC Endpoint Address Out Transfer : 0x02
uint8_t NFC_EPP_IN1 = 0x01; //NFC Endpoint Address In Transfer 1: 0x81
uint8_t NFC_EPP_IN2 = 0x02; //NFC Endpoint Address In Transfer 2: 0x82

uint8_t NFC_Easy_Connect_Code[15] =  {0x6B,0x05,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0xFF,0x00,0x48,0x00,0x00};
uint8_t NFC_Reader_Code[10] =  {0x62,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00};
uint8_t NFC_Get_UID_Code[15] =  {0x6F,0x05,0x00,0x00,0x00,0x00,0x03,0x04,0x00,0x00,0xFF,0xCA,0x00,0x00,0x00};

  
void setup() {
    if (Usb.Init() == -1){
      //Near Field Communication (NFC) USB Reader did not start
      while(1);   
    }
   
  FX1S_construct(FX1Spacket1, FX1S_WRITE_D, 0, NFC_Tags_Max+1, NFC_FX1SwriteD);

  FX1S_configure(&Serial, FX1Sbaud, FX1Sformat, FX1Stimeout, FX1Spackets, FX1STOTAL_NO_OF_PACKETS);
  
  NFC_USB_State=0;
  NFC_USB_Order=1;
  NFC_Receive_String.reserve(64);
}

void loop() {  
  Usb.Task();
  if( Usb.getUsbTaskState() == USB_STATE_RUNNING )
  {
  NFC_USB_RUN = true;

            switch(NFC_USB_State) {
                    case 0:  // NFC USB Configuration                  
                            NFC_Code_Return = ACR122U_NFC_Configuration();
                            if(NFC_Code_Return){
                              asm volatile ("  jmp 0");
                            }else{
                              NFC_USB_State=1;
                            }
                            break;

                            
                      case 1: // ACR122U Open NFC or NFC Connect
                      NFC_USB_Order++;                                      
                      NFC_Code_Return = ACR122U_Open_NFC();                  
                      if(NFC_Code_Return){
                        NFC_USB_State=1;
                      }else{
                        NFC_USB_State=2;
                      }
                      break;
                      

                    case 2: // ACR122U USB NFC Setup
                      NFC_USB_Order++; 
                      NFC_Code_Return = ACR122U_USB_NFC_Reader();                   
                      if(NFC_Code_Return){
                        NFC_USB_State=2;
                      }else{
                        NFC_USB_State=3;
                      }
                      break;
                      
                      
                    case 3: // ACR122U USB NFC Tag Reader
                        NFC_Receive_Size=0;
                        NFC_Receive[0]=0;
                        NFC_Receive[1]=0;
                        NFC_Code_Return = ACR122U_USB_NFC_Read(NFC_EPP_IN1,&NFC_Receive_Size, NFC_Receive, 1);
                        if(NFC_Code_Return){
                          NFC_USB_State=3;
                        }else{
                          if(NFC_Receive_Size==2){
                            if(NFC_Receive[0]==0x50 && NFC_Receive[1]==0x03 ){
                              NFC_Get_UID_Timeout = millis() + 100;
                              NFC_USB_State=4;
                            }
                          }
                        }
                        break;
                        
                      
                    case 4: // ACR122U NFC Get Data UID
                      NFC_USB_Order++;
                      NFC_Tags_Size=0;
                      NFC_Code_Return = ACR122U_USB_NFC_Tag_Reader(&NFC_Tags_Size, NFC_Tags);
                      if(NFC_Code_Return){
                        if(millis() > NFC_Get_UID_Timeout)NFC_USB_State=3;
                      }else{
                        if(NFC_Tags_Size>0){
                          memset(NFC_FX1SwriteD,0,sizeof(NFC_FX1SwriteD));
                          NFC_FX1SwriteD[0]=NFC_Tags_Size;
                          for(uint16_t i=0; i < NFC_Tags_Size; i++ ) {
                          NFC_FX1SwriteD[i+1]= NFC_Tags[i];
                          }
                          NFC_Send_Success = FX1Spacket1->FX1Ssuccessful_requests;
                          NFC_Get_UID_Timeout = millis() + 1000;
                          NFC_USB_State=5;
                        }else{
                          if(millis() > NFC_Get_UID_Timeout)NFC_USB_State=3;
                        }
                      }
                      break;  


                    case 5: // NFC UID Send to Mitsubishi PLC FX
                      FX1S_update();
                      if(NFC_Send_Success != FX1Spacket1->FX1Ssuccessful_requests)NFC_USB_State=3;
                      if(millis() > NFC_Get_UID_Timeout)NFC_USB_State=3;
                      break;
                                       
            }
            
  }else{
    if(NFC_USB_RUN){
      // ACR122U NFC USB Not RUN
      asm volatile ("  jmp 0"); 
    }
  }


}



void NFC_USB_GetAddresses(UsbDevice *pdev)
{
    UsbDeviceAddress NFC_USB_addr;
    NFC_USB_addr.devAddress = pdev->address.devAddress;
    NFC_USB_Address = NFC_USB_addr.devAddress;
}

uint8_t ACR122U_NFC_Configuration() {
  uint8_t Rcode;
  Usb.ForEachUsbDevice(&NFC_USB_GetAddresses);
  Rcode = Usb.getDevDescr(NFC_USB_Address, 0, sizeof (USB_DEVICE_DESCRIPTOR), (uint8_t*) & buf);
  if (Rcode) {
    return (Rcode);
  }else{
    Rcode = Usb.setConf(NFC_USB_Address, 0, buf.bNumConfigurations);
    return (Rcode);            
  }        
  return (USB_STATE_ERROR);
}




uint8_t ACR122U_Open_NFC() { 
 uint8_t Rcode;  
 NFC_Easy_Connect_Code[6]=NFC_USB_Order;
 Rcode =  ACR122U_USB_NFC_Write(NFC_EPP_OUT,sizeof(NFC_Easy_Connect_Code),NFC_Easy_Connect_Code); 
 if(Rcode)
    return Rcode;

  NFC_Receive_Total_Data=0;
  NFC_Receive[0]=0;
  NFC_Receive[1]=0;
  Rcode = ACR122U_USB_NFC_Read(NFC_EPP_IN2,&NFC_Receive_Total_Data, NFC_Receive, 10);  
  if(Rcode)
    return Rcode; 

  if(NFC_Receive_Total_Data<12)
    return (1);

   if(NFC_Receive[0]!=0x83 || NFC_Receive[1]!=0x0A)
     return (1);     
  
  NFC_Receive_String="";
  for(uint16_t i=7; i < NFC_Receive_Total_Data; i++ )
    NFC_Receive_String += String(char(NFC_Receive[i]));   

  if (NFC_Receive_String.indexOf(NFC_Hardware)!=-1)
    return (0); 

return (1);        
}



uint8_t ACR122U_USB_NFC_Reader() {
 uint8_t Rcode;  
 NFC_Reader_Code[6]=NFC_USB_Order;
 Rcode =  ACR122U_USB_NFC_Write(NFC_EPP_OUT,sizeof(NFC_Reader_Code),NFC_Reader_Code); 
 if(Rcode)
    return Rcode;

  NFC_Receive_Total_Data=0;
  NFC_Receive[0]=0;
  Rcode = ACR122U_USB_NFC_Read(NFC_EPP_IN2,&NFC_Receive_Total_Data, NFC_Receive, 10);   
  if(Rcode)
    return Rcode; 

  if(NFC_Receive_Total_Data<12)
    return (1);

   if(NFC_Receive[0]==0x80)
     return (0);
     
return (1);
}



uint8_t ACR122U_USB_NFC_Tag_Reader(uint16_t *NFC_Key_Size, uint8_t* NFC_Key) {
  
 *NFC_Key_Size=0; 
 uint8_t Rcode;  
 NFC_Get_UID_Code[6]=NFC_USB_Order;
 Rcode =  ACR122U_USB_NFC_Write(NFC_EPP_OUT,sizeof(NFC_Get_UID_Code),NFC_Get_UID_Code); 
 if(Rcode)
    return Rcode;


  NFC_Receive_Total_Data=0;
  NFC_Receive[0]=0;
  NFC_Receive[1]=0;
  Rcode = ACR122U_USB_NFC_Read(NFC_EPP_IN2,&NFC_Receive_Total_Data, NFC_Receive, 50);  
  if(Rcode)
    return Rcode; 

  if(NFC_Receive_Total_Data<12)
    return (1);

   uint16_t Buffer_NFC_Key_Size = 0;      
   if(NFC_Receive[0]==0x80 && NFC_Receive[1]>0x02){      
      for(uint16_t i=10; i < (NFC_Receive_Total_Data-2); i++ ) {
      *NFC_Key++ = NFC_Receive[i];
      Buffer_NFC_Key_Size++; 
      }
      
      *NFC_Key_Size = Buffer_NFC_Key_Size;       
     return (0);
   }

return (1);
}



uint8_t ACR122U_USB_NFC_Write(uint8_t vHXFR,uint16_t nbytes, uint8_t* data) {
    Usb.bytesWr(rSNDFIFO, nbytes, data);
    Usb.regWr(rSNDBC, nbytes);
    Usb.regWr(rHXFR, vHXFR);
    while(!(Usb.regRd(rHIRQ) & bmHXFRDNIRQ));
    Usb.regWr(rHIRQ, bmHXFRDNIRQ);       
  return (0);
}


uint8_t ACR122U_USB_NFC_Read(uint8_t vHXFR,uint16_t *pktsize, uint8_t* data, unsigned long timeout) {
  unsigned long timeout_start = millis() + timeout;
    
  while((long)(millis() - timeout_start) < 0L) {
    Usb.regWr(rHXFR, vHXFR);
      if((Usb.regRd(rHIRQ) & bmRCVDAVIRQ)==bmRCVDAVIRQ){
      uint16_t buff_pktsize = Usb.regRd(rRCVBC);
      *pktsize = buff_pktsize;
      data = Usb.bytesRd(rRCVFIFO, buff_pktsize, data);
      Usb.regWr(rHIRQ, bmRCVDAVIRQ);
      return (0);   
      }   
   }      
             
  return (1);
}
