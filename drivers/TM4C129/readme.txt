Link into utils folder:
 TivaWare_C_Series-2.1.4.178\utils\locator.c (if needed - enable in ethernet/enet.c)
 TivaWare_C_Series-2.1.4.178\utils\lwiplib.c

Link into third-party folder:
 TivaWare_C_Series-2.1.4.178\third_party\fatfs
  Exclude from build:
   all files in fatfs/port except mmc-ek-tm4c1294xl.c 
   all files in fatfs/src/option except syscall.c and unicode.c
 TivaWare_C_Series-2.1.4.178\third_party\FreeRTOS
 TivaWare_C_Series-2.1.4.178\third_party\lwip-1.4.1
