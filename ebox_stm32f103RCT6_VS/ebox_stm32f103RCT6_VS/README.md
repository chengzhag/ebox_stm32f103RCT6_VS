# ebox_stm32
# ebox_stm32特点
1. 在STM32的3.5版本的库的基础上封装一层类似于Arduino的API。
1. 使得STM32也可以使用arduino的大部分驱动。驱动程序从github上下载后稍作修改就可以使用。
1. 减少STM32开发人员编写、调试器件驱动的工作量，提高驱动的重复利用率。

#如何编译
1.使用MDK 515编译
1.已将工程搬移到visiual studio+visualGDB

#如何将工程搬移到visualGDB上？
> https://visualgdb.com/tutorials/arm/keil/

## 导入keil工程目录
1. 文件扩展名必须小写
1. 导入工程文件夹时去掉原工程没有导入的c/cpp文件，避免include未设置在包含路径内的h文件

## 设置编译器、链接器flag
1. COMMONFLAGS去掉```--split_sections```，否则会报错
1. 改为```--list "$(BINARYDIR)\ebox.map"```，将输出内容保存到Debug文件夹
1. 去掉```--scatter ".\Objects\ebox.sct"```，否则会报错
1. LDFLAGS需要添加```--ro-base```、```--rw-base```等flag，这些设置在keil的项目设置的linker选项卡下

## 设置包含目录
1. 项目的visualGDB propoties的makefile settings中也需要设置include，否则intellisense无法正常
1. 在需要链接dll文件的情况下，需要再makefile settings中设置additional linker inputs

## 设置debug settings
1. 配置j-link时注意手动选择设备类型
1. 下载报错：```Cannot resolve the address of _estack. Skipping stack pointer validity check.```，可以在debug settings中取消勾选validate stack pointer when starting debugging选项解决。

## 其他
1. 全局变量初始化为-1导致程序运行失败，要按照教程后修改Makefile，将```ifeq ($(TARGETTYPE),APP)...endif```替换为
```
ifeq ($(TARGETTYPE),APP)
ROM_SECTION_NAME := ER_RO
$(BINARYDIR)/$(TARGETNAME): $(all_objs) $(EXTERNAL_LIBS)
    $(LD) -o $(@:.elf=.axf) $(LDFLAGS) $(START_GROUP) $(all_objs) \
      $(LIBRARY_LDFLAGS) $(END_GROUP)
    $(FROMELF) --bin --output $(@:.elf=.bin) $(@:.elf=.axf)
    $(OBJCOPY) --remove-section $(ROM_SECTION_NAME) \
        --add-section $(ROM_SECTION_NAME)=$(@:.elf=.bin) \
        --set-section-flags \
        $(ROM_SECTION_NAME)=CONTENTS,ALLOC,LOAD,READONLY,CODE \
        --change-section-address $(ROM_SECTION_NAME)=0x08000000 \
        $(@:.elf=.axf) $@
endif
```
1. makefile,.mak文件中使用```\```进行换行时，后面不能有包括空格在内的任何字符
