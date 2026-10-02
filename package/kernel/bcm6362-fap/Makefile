#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=bcm6362-fap
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-or-later

include $(INCLUDE_DIR)/package.mk

define KernelPackage/bcm6362-fap
  SUBMENU:=Other modules
  TITLE:=BCM6362 FAP coprocessor loader
  DEPENDS:=@TARGET_bmips
  FILES:=$(PKG_BUILD_DIR)/bcm6362-fap.ko
endef

define KernelPackage/bcm6362-fap/description
 Boots a custom firmware on the MIPS 4KE coprocessor (FAP) of the BCM6362
 and exposes its log and mailbox through debugfs.
endef

define Build/Compile
	$(KERNEL_MAKE) M="$(PKG_BUILD_DIR)" modules
	$(MAKE) -C $(PKG_BUILD_DIR)/firmware CROSS_COMPILE="$(TARGET_CROSS)"
endef

define KernelPackage/bcm6362-fap/install
	$(INSTALL_DIR) $(1)/lib/firmware
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/firmware/bcm6362-fap.bin $(1)/lib/firmware/
endef

$(eval $(call KernelPackage,bcm6362-fap))
