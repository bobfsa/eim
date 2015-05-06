#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xa06cb60e, "module_layout" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x1d7d4310, "platform_driver_probe" },
	{ 0xf9a482f9, "msleep" },
	{ 0x67c2fa54, "__copy_to_user" },
	{ 0x2e5810c6, "__aeabi_unwind_cpp_pr1" },
	{ 0x47229b5c, "gpio_request" },
	{ 0xcb136961, "device_destroy" },
	{ 0x2e1ca751, "clk_put" },
	{ 0x338f065a, "__register_chrdev" },
	{ 0x1099eee8, "clk_get_rate" },
	{ 0xfa2a45e, "__memzero" },
	{ 0x5f754e5a, "memset" },
	{ 0x65d6d0f0, "gpio_direction_input" },
	{ 0x74c97f9c, "_raw_spin_unlock_irqrestore" },
	{ 0x27e1a049, "printk" },
	{ 0xdb3877d, "___dma_single_dev_to_cpu" },
	{ 0xfaef0ed, "__tasklet_schedule" },
	{ 0x9545af6d, "tasklet_init" },
	{ 0xd3d57d53, "device_create" },
	{ 0xb1d7c92, "platform_device_unregister" },
	{ 0xd6b8e852, "request_threaded_irq" },
	{ 0x67c02674, "platform_device_register" },
	{ 0x93fca811, "__get_free_pages" },
	{ 0xbd7083bc, "_raw_spin_lock_irqsave" },
	{ 0x719e394c, "clk_get" },
	{ 0x4302d0eb, "free_pages" },
	{ 0xfe990052, "gpio_free" },
	{ 0xafc37f0f, "dma_release_channel" },
	{ 0x80842341, "devm_ioremap" },
	{ 0x680ce549, "class_destroy" },
	{ 0x45a55ec8, "__iounmap" },
	{ 0x40a6f522, "__arm_ioremap" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
	{ 0xef9720dc, "devm_iounmap" },
	{ 0xca54fee, "_test_and_set_bit" },
	{ 0xdfebe27c, "platform_driver_unregister" },
	{ 0x28e56a8e, "__class_create" },
	{ 0xf20dabd8, "free_irq" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

