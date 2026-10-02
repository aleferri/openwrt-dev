include $(TOPDIR)/rules.mk

PKG_NAME:=b43-jandtest
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-or-later

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define KernelPackage/b43-jandtest
  SUBMENU:=Other modules
  TITLE:=Broadcom d11 PSM jand/jnand polarity test
  DEPENDS:=+kmod-bcma
  FILES:=$(PKG_BUILD_DIR)/b43-jandtest.ko
  AUTOLOAD:=
endef

define KernelPackage/b43-jandtest/description
  One-shot test kernel module. Binds to every 802.11 core on the bcma
  buses, uploads b43/jandtest{5,15}.fw and runs it once per operand
  pair (0,0), (0,1), (1,0), (1,1), logging which of jand and jnand
  jumped. Refuses to stay bound; unload b43 first, each insmod replays
  the test in dmesg. files/jandtest.asm is the source of the firmware.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(KERNEL_MAKE) M="$(PKG_BUILD_DIR)" modules
endef

define KernelPackage/b43-jandtest/install
	$(INSTALL_DIR) $(1)/lib/firmware/b43
	$(INSTALL_DATA) ./files/jandtest5.fw ./files/jandtest15.fw $(1)/lib/firmware/b43/
endef

$(eval $(call KernelPackage,b43-jandtest))
