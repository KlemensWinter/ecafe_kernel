#------------------------------------------------------------------------------
# <copyright file="makefile" company="Ralink">
#
#------------------------------------------------------------------------------
#==============================================================================
# Author(s): ="Guillemot Corporation"
#==============================================================================

include $(obj)/rt3070.mk


EXTRA_CFLAGS += -DSYSTEM_LOG_SUPPORT -DLED_CONTROL_SUPPORT -DLINUX
EXTRA_CFLAGS +=-DRTMP_MAC_USB -DRT30xx -DRT3070 -DRTMP_USB_SUPPORT -DRTMP_TIMER_TASK_SUPPORT -DRTMP_RF_RW_SUPPORT -DRTMP_EFUSE_SUPPORT -I$(obj)/include
CHIPSET_DAT = 2870

ifeq ($(HAS_RESOURCE_PRE_ALLOC),y)
EXTRA_CFLAGS += -DRESOURCE_PRE_ALLOC
endif

ifeq ($(HAS_KTHREAD_SUPPORT),y)
EXTRA_CFLAGS += -DKTHREAD_SUPPORT
endif

ifeq ($(HAS_RTMP_FLASH_SUPPORT),y)
EXTRA_CFLAGS += -DRTMP_FLASH_SUPPORT
endif


#################################################

# config for STA mode

ifeq ($(RT28xx_MODE),STA)
#EXTRA_CFLAGS += -DCONFIG_STA_SUPPORT -DDBG
EXTRA_CFLAGS += -DCONFIG_STA_SUPPORT


ifeq ($(HAS_XLINK),y)
EXTRA_CFLAGS += -DXLINK_SUPPORT
endif


ifeq ($(HAS_WPA_SUPPLICANT),y)
EXTRA_CFLAGS += -DWPA_SUPPLICANT_SUPPORT
ifeq ($(HAS_NATIVE_WPA_SUPPLICANT_SUPPORT),y)
EXTRA_CFLAGS += -DNATIVE_WPA_SUPPLICANT_SUPPORT
endif
endif




ifeq ($(HAS_ATE),y)
EXTRA_CFLAGS += -DRALINK_ATE
EXTRA_CFLAGS += -DCONFIG_RT2880_ATE_CMD_NEW
ifeq ($(HAS_NEW_TXCONT),y)
EXTRA_CFLAGS += -DNEW_TXCONT
endif
ifeq ($(HAS_QA_SUPPORT),y)
EXTRA_CFLAGS += -DRALINK_QA
endif
endif


ifeq ($(HAS_SNMP_SUPPORT),y)
EXTRA_CFLAGS += -DSNMP_SUPPORT
endif

ifeq ($(HAS_QOS_DLS_SUPPORT),y)
EXTRA_CFLAGS += -DQOS_DLS_SUPPORT
endif

ifeq ($(HAS_DOT11_N_SUPPORT),y)
EXTRA_CFLAGS += -DDOT11_N_SUPPORT
ifeq ($(HAS_DOT11N_DRAFT3_SUPPORT),y)
EXTRA_CFLAGS += -DDOT11N_DRAFT3
endif
endif


ifeq ($(HAS_CS_SUPPORT),y)
EXTRA_CFLAGS += -DCARRIER_DETECTION_SUPPORT
endif

ifeq ($(HAS_STATS_COUNT),y)
EXTRA_CFLAGS += -DSTATS_COUNT_SUPPORT
endif

ifeq ($(HAS_CFG80211_SUPPORT),y)
EXTRA_CFLAGS += -DRT_CFG80211_SUPPORT
ifeq ($(HAS_RFKILL_SUPPORT),y)
EXTRA_CFLAGS += -DRFKILL_SUPPORT
endif
endif

ifeq ($(OSABL),YES)
EXTRA_CFLAGS += -DOS_ABL_SUPPORT
endif

ifeq ($(HAS_ANTENNA_DIVERSITY_SUPPORT),y)
EXTRA_CFLAGS += -DANT_DIVERSITY_SUPPORT
endif


endif
# endif of ifeq ($(RT28xx_MODE),STA)

#################################################

#################################################

#
# Common compiler flag
#

ifeq ($(HAS_EXT_BUILD_CHANNEL_LIST),y)
EXTRA_CFLAGS += -DEXT_BUILD_CHANNEL_LIST
endif

ifeq ($(HAS_IDS_SUPPORT),y)
EXTRA_CFLAGS += -DIDS_SUPPORT
endif


ifeq ($(OSABL),YES)
EXTRA_CFLAGS += -DEXPORT_SYMTAB
endif

ifeq ($(HAS_CLIENT_WDS_SUPPORT),y)
EXTRA_CFLAGS += -DCLIENT_WDS
endif

ifeq ($(HAS_BGFP_SUPPORT),y)
EXTRA_CFLAGS += -DBG_FT_SUPPORT
endif

ifeq ($(HAS_BGFP_OPEN_SUPPORT),y)
EXTRA_CFLAGS += -DBG_FT_OPEN_SUPPORT
endif


ifeq ($(RT28xx_MODE), STA)
MOD_NAME = rt$(CHIPSET)sta
DAT_PATH = /etc/Wireless/RT$(CHIPSET_DAT)STA
DAT_FILE_NAME = RT$(CHIPSET_DAT)STA.dat
endif
#endif // CONFIG_STA_SUPPORT //


obj-m := $(MOD_NAME).o



#ifdef CONFIG_STA_SUPPORT
obj-$(CONFIG_RT3070STA) := rt3070sta.o
rt3070sta-y += common/crypt_md5.o
rt3070sta-y += common/crypt_sha2.o
rt3070sta-y += common/crypt_hmac.o
rt3070sta-y += common/crypt_aes.o
rt3070sta-y += common/crypt_arc4.o
rt3070sta-y += common/mlme.o
rt3070sta-y += common/cmm_wep.o
rt3070sta-y += common/action.o
rt3070sta-y += common/cmm_data.o
rt3070sta-y += common/rtmp_init.o
rt3070sta-y += common/cmm_tkip.o
rt3070sta-y += common/cmm_aes.o
rt3070sta-y += common/cmm_sync.o
rt3070sta-y += common/eeprom.o
rt3070sta-y += common/cmm_sanity.o
rt3070sta-y += common/cmm_info.o
rt3070sta-y += common/cmm_cfg.o
rt3070sta-y += common/cmm_wpa.o
rt3070sta-y += common/dfs.o
rt3070sta-y += common/spectrum.o
rt3070sta-y += common/rtmp_timer.o
rt3070sta-y += common/rt_channel.o
rt3070sta-y += common/cmm_profile.o
rt3070sta-y += common/cmm_asic.o
rt3070sta-y += common/cmm_cmd.o
rt3070sta-y += sta/assoc.o
rt3070sta-y += sta/auth.o
rt3070sta-y += sta/auth_rsp.o
rt3070sta-y += sta/sync.o
rt3070sta-y += sta/sanity.o
rt3070sta-y += sta/rtmp_data.o
rt3070sta-y += sta/connect.o
rt3070sta-y += sta/wpa.o
rt3070sta-y += sta/ags.o
rt3070sta-y += sta/sta_cfg.o

ifeq ($(OSABL),NO)
rt3070sta-y += common/rtmp_init_inf.o
rt3070sta-y += os/linux/rt_profile.o
rt3070sta-y += os/linux/sta_ioctl.o
rt3070sta-y += os/linux/rt_linux.o
rt3070sta-y += os/linux/rt_main_dev.o
endif

#ifdef DOT11_N_SUPPORT
ifeq ($(HAS_DOT11_N_SUPPORT),y)
rt3070sta-y += common/ba_action.o
endif
#endif // DOT11_N_SUPPORT //

#ifdef ETH_CONVERT
ifeq ($(HAS_ETH_CONVERT_SUPPORT), y)
rt3070sta-y += common/cmm_mat.o
rt3070sta-y += common/cmm_mat_iparp.o
rt3070sta-y += common/cmm_mat_pppoe.o
rt3070sta-y += common/cmm_mat_ipv6.o
endif
#endif // ETH_CONVERT //

ifeq ($(HAS_BLOCK_NET_IF),y)
rt3070sta-y += common/netif_block.o
endif

ifeq ($(HAS_QOS_DLS_SUPPORT),y)
rt3070sta-y += sta/dls.o
endif

ifeq ($(CHIPSET),3070)
rt3070sta-y += common/cmm_mac_usb.o
rt3070sta-y += common/rtusb_io.o
rt3070sta-y += common/rtusb_bulk.o
rt3070sta-y += common/rtusb_data.o
rt3070sta-y += common/cmm_data_usb.o
rt3070sta-y += common/ee_prom.o
rt3070sta-y += common/ee_efuse.o
rt3070sta-y += common/rtmp_mcu.o
rt3070sta-y += chips/rt30xx.o
rt3070sta-y += common/rt_rf.o
rt3070sta-y += chips/rt3070.o

	ifeq ($(OSABL),NO)
	rt3070sta-y += common/rtusb_dev_id.o
	rt3070sta-y += os/linux/rt_usb.o
	rt3070sta-y += os/linux/rt_usb_util.o
	rt3070sta-y += os/linux/usb_main_dev.o
	endif
endif



