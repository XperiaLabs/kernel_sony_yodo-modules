AKS_JS_DLKM_ENABLE := true
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	AKS_JS_DLKM_ENABLE := false
endif

ifeq ($(AKS_JS_DLKM_ENABLE),  true)
	PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/aks-adc-joystick.ko \
		$(KERNEL_MODULES_OUT)/aks-ads1015.ko
endif
