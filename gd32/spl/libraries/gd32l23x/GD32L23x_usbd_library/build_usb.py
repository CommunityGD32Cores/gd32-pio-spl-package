from os.path import join, realpath
Import("env")

# map flags to their class name (which we can deduct the path from)
device_flags_to_sources = {
    "PIO_USBFS_DEVICE_AUDIO": "audio",
    "PIO_USBFS_DEVICE_CDC": "cdc",
    "PIO_USBFS_DEVICE_DFU": "dfu",
    "PIO_USBFS_DEVICE_HID_STANDARD": "hid",
    "PIO_USBFS_DEVICE_HID_CUSTOM": "hid",
    "PIO_USBFS_DEVICE_IAP": "iap",
    "PIO_USBFS_DEVICE_MSC": "msc",
    "PIO_USBFS_DEVICE_PRINTER": "printer"
}

device_flags_to_special_excludes = {
    "PIO_USBFS_DEVICE_HID_STANDARD": join("class", "device", "hid", "Source", "custom_hid_core.c"),
    "PIO_USBFS_DEVICE_HID_CUSTOM": join("class", "device", "hid", "Source", "standard_hid_core.c"),
}

include_parts = []
source_parts = []
include_device_core = False
special_excludes = []

# source through CPPDEFINES for config flags
for item in env.Flatten(env.get("CPPDEFINES", [])):
    item = str(item)
    #print("Got item %s" % str(item))
    if item in device_flags_to_sources.keys():
        include_device_core = True
        class_name = device_flags_to_sources[item]
        print("USBFS option %s found, adding class \"%s\"" %
              (str(item), class_name))
        include_parts.append(join("class", "device", class_name, "Include"))
        source_parts.append(join("class", "device", class_name, "Source"))
        if item in device_flags_to_special_excludes.keys():
            special_excludes.append(device_flags_to_special_excludes[item])

if include_device_core:
    include_parts.append(join("device", "Include"))
    source_parts.append(join("device", "Source"))

# global build environment is for all libraries and code
global_env = DefaultEnvironment()

for inc in include_parts:
    env.Append(CPPPATH=[realpath(inc)])
    global_env.Append(CPPPATH=[realpath(inc)])

# build one source filter expressin
src_filter_default = [
    "-<*>", # exclude everything by default
    "+<%s*>" % join("usbd", "Source"), # include the entire driver code (host + device), special exclusions apply.
]

for src in source_parts:
    src_filter_default.append(
        "+<%s*>" % src
    )

for excl in special_excludes:
    src_filter_default.append(
        "-<%s>" % excl
    )

print("USBFS source filter excludes: %s" % str(src_filter_default))

env.Replace(SRC_FILTER=src_filter_default)

# usbd_lld_core.c can use this macro to use the internal clock
#global_env.Append(CPPDEFINES=["USE_IRC48M"])
#env.Append(CPPDEFINES=["USE_IRC48M"])
