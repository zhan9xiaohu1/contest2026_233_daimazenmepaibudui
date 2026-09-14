这是一个占位文件，用来保证 /etc/assets 目录在仓库里存在。

放演示素材（提示音、图片、模型、配置文件等任何只读数据）的方法：
    把文件直接拷进  board/contest_board/src/etc/assets/
然后重新编译 + 烧录即可。

上板后它们出现在：  /etc/assets/<你的文件名>
权限是只读（-r--r--r--），因为整个 /etc 是一个 ROMFS 镜像，
编译时被打进固件里了（genromfs → romfs_etc.c → 链接进 nuttx.bin）。


两个特殊文件（都是本机生成、**不进版本库**，见 .git/info/exclude）：

  push_key.txt        手机推送的 device key，运行时被 network_comm.c 读走。
  agent_config.json   ai_agent 的凭据（大模型 key、火山 ASR/TTS 凭据）。
                      开机时由板级 sf32lb52_install_agent_config() 拷到
                      /data/ai_agent/config/config.json —— 因为 /data 是 tmpfs，
                      重启就清空，不拷的话每次都要手敲 set_llm / set_volc_*。
                      没有这个文件也能正常开机（跳过即可，固件里不含任何密钥）。
详见 docs/rom_assets_usage.md


唤醒词模板（kws/ 子目录）—— 想让唤醒词开机就能用，看这一节

唤醒词「你好，openvela」/「Hello，openvela」的模板是 MFCC 特征点，运行时
存在 /data/kws/slotN.tpl（N = 0..3，slot0 = 中文词、slot1 = 英文词）。
而 /data 是 tmpfs，**重启就清空** —— 不放素材的话，唤醒词每次重启都静默失效，
只剩 VAD 触发（说话还是会进 ASR，只是不能靠喊名字唤醒）。

想让唤醒词"开机即用"，就把录好的模板放进这个目录：

    board/contest_board/src/etc/assets/kws/slot0.tpl
    board/contest_board/src/etc/assets/kws/slot1.tpl
                                ↑ 没有这一条就不放，缺哪个放哪个都行

行为（实现在 app/hello_app/kws_dtw.c 的 kws_install_templates()，
套路同上面 agent_config.json 那一节）：

  - 开机 kws_init() 时把 /etc/assets/kws/slotN.tpl 拷成 /data/kws/slotN.tpl，
    再走正常的载入路径 —— 拷出来的和现场录的是同一种文件，不区分；
  - **目标已存在就不覆盖**：现场用 hw_test 录的模板（更贴合说话人）优先；
  - 目录里没有素材（仓库的默认状态）就安静跳过：不打 ERROR、不阻塞、
    不影响启动，只是唤醒词仍然用不了；
  - 串口日志只有一行字节数："[KWS] 出厂模板补装到 /data/kws：1714 字节（1 条）"。

怎么得到 .tpl：在板子上用 NSH 录一条，落盘就是素材本体

    nsh> hw_test kws enroll 0        # 对着板子说「你好，openvela」，落盘
    nsh> hw_test kws enroll 1        # 再说「Hello，openvela」
    nsh> ls -l /data/kws             # 看到 slot0.tpl / slot1.tpl（各约 1.7KB）

然后把 /data/kws/slotN.tpl 取到 PC（NSH 里现在没有专门的导出命令，用你们惯用的
串口/网络办法把这两个小文件捞出来），放进 assets/kws/ 重新编译烧录。
数据格式就是 24 字节头 + int16 特征点（见 kws_dtw.h 文件头），
模板换了特征口径（改 KWS_* 常量）时必须重录，旧文件会被版本检查挡掉。

⚠ 两个必须注意的点：

  1) **必须删掉 ROMFS 产物再编**。`nuttx_add_romfs` 那条自定义命令的 DEPENDS
     只跟踪逐个列出的文件，PATH 目录里文件的增删它感知不到（改了 etc/ 下的
     文件也一样）。删 `<build>/boards/exclude_board/src/romfs.img`、
     `romfs_etc.c`、`romfs_etc/` 之后重编才会把新素材打进去。
     详见 docs/rom_assets_usage.md 的"排查"一节。
  2) 空目录不进 ROMFS：assets/kws/ 里**至少要有 .tpl 文件**，
     genromfs 不会生成空目录（也就是不能靠"先建个空目录"占位）。

