.section .data.configdefault,"a"
.global _binary_config_default_start
_binary_config_default_start:
.incbin "config.default"
_binary_config_default_end:
.global _binary_config_default_end
