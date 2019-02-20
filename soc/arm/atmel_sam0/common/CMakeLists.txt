zephyr_include_directories(.)
zephyr_sources_ifdef(CONFIG_SYS_POWER_MANAGEMENT power.c)
