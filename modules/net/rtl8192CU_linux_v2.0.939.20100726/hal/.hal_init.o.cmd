cmd_/home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/hal/hal_init.o := arm-none-linux-gnueabi-gcc -Wp,-MD,/home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/hal/.hal_init.o.d  -nostdinc -isystem /home/andy/CodeSourcery/Sourcery_G++_Lite/bin/../lib/gcc/arm-none-linux-gnueabi/4.3.3/include -I/home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include -Iinclude  -include include/generated/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-davinci/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Os -marm -fno-omit-frame-pointer -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -D__LINUX_ARM_ARCH__=5 -march=armv5te -mtune=arm9tdmi -msoft-float -Uarm -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -O1 -Wno-unused-variable -Wno-unused-value -Wno-unused-label -Wno-unused-parameter -Wno-uninitialized -Wno-unused -Wno-unused-function -I/home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include -DCONFIG_LITTLE_ENDIAN  -DMODULE -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(hal_init)"  -D"KBUILD_MODNAME=KBUILD_STR(8192cu)"  -c -o /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/hal/.tmp_hal_init.o /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/hal/hal_init.c

deps_/home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/hal/hal_init.o := \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/hal/hal_init.c \
    $(wildcard include/config/sdio/hci.h) \
    $(wildcard include/config/usb/hci.h) \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/drv_conf.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/autoconf.h \
    $(wildcard include/config/debug/rtl871x.h) \
    $(wildcard include/config/pcie/hci.h) \
    $(wildcard include/config/rtl8711.h) \
    $(wildcard include/config/rtl8712.h) \
    $(wildcard include/config/rtl8192c.h) \
    $(wildcard include/config/rtl8192d.h) \
    $(wildcard include/config/little/endian.h) \
    $(wildcard include/config/big/endian.h) \
    $(wildcard include/config/pwrctrl.h) \
    $(wildcard include/config/h2clbk.h) \
    $(wildcard include/config/mp/included.h) \
    $(wildcard include/config/embedded/fwimg.h) \
    $(wildcard include/config/r871x/test.h) \
    $(wildcard include/config/80211n/ht.h) \
    $(wildcard include/config/recv/reordering/ctrl.h) \
    $(wildcard include/config/rtl8712/tcp/csum/offload/rx.h) \
    $(wildcard include/config/drvext.h) \
    $(wildcard include/config/ips.h) \
    $(wildcard include/config/lps.h) \
    $(wildcard include/config/pm.h) \
    $(wildcard include/config/bt/coexist.h) \
    $(wildcard include/config/antenna/diversity.h) \
    $(wildcard include/config/debug/rtl8192c.h) \
    $(wildcard include/config/only/one/out/ep/to/low.h) \
    $(wildcard include/config/out/ep/wifi/mode.h) \
    $(wildcard include/config/prealloc/recv/skb.h) \
    $(wildcard include/config/reduce/usb/tx/int.h) \
    $(wildcard include/config/ap/mode.h) \
    $(wildcard include/config/nativeap/mlme.h) \
    $(wildcard include/config/hostapd/mlme.h) \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/osdep_service.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/basic_types.h \
  include/linux/types.h \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/types.h \
  include/asm-generic/int-ll64.h \
  include/asm-generic/bitsperlong.h \
  include/linux/posix_types.h \
  include/linux/stddef.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/posix_types.h \
  include/linux/version.h \
  include/linux/spinlock.h \
    $(wildcard include/config/smp.h) \
    $(wildcard include/config/debug/spinlock.h) \
    $(wildcard include/config/generic/lockbreak.h) \
    $(wildcard include/config/preempt.h) \
    $(wildcard include/config/debug/lock/alloc.h) \
  include/linux/typecheck.h \
  include/linux/preempt.h \
    $(wildcard include/config/debug/preempt.h) \
    $(wildcard include/config/preempt/tracer.h) \
    $(wildcard include/config/preempt/notifiers.h) \
  include/linux/thread_info.h \
    $(wildcard include/config/compat.h) \
  include/linux/bitops.h \
    $(wildcard include/config/generic/find/first/bit.h) \
    $(wildcard include/config/generic/find/last/bit.h) \
    $(wildcard include/config/generic/find/next/bit.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/bitops.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/system.h \
    $(wildcard include/config/cpu/xsc3.h) \
    $(wildcard include/config/cpu/fa526.h) \
    $(wildcard include/config/cpu/sa1100.h) \
    $(wildcard include/config/cpu/sa110.h) \
    $(wildcard include/config/cpu/32v6k.h) \
  include/linux/linkage.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/linkage.h \
  include/linux/irqflags.h \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/irqsoff/tracer.h) \
    $(wildcard include/config/trace/irqflags/support.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/irqflags.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/arm/thumb.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/hwcap.h \
  include/asm-generic/cmpxchg-local.h \
  include/asm-generic/cmpxchg.h \
  include/asm-generic/bitops/non-atomic.h \
  include/asm-generic/bitops/fls64.h \
  include/asm-generic/bitops/sched.h \
  include/asm-generic/bitops/hweight.h \
  include/asm-generic/bitops/lock.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/thread_info.h \
    $(wildcard include/config/arm/thumbee.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/fpstate.h \
    $(wildcard include/config/vfpv3.h) \
    $(wildcard include/config/iwmmxt.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/domain.h \
    $(wildcard include/config/io/36.h) \
    $(wildcard include/config/mmu.h) \
  include/linux/list.h \
    $(wildcard include/config/debug/list.h) \
  include/linux/poison.h \
    $(wildcard include/config/illegal/pointer/value.h) \
  include/linux/prefetch.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/processor.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/cache.h \
    $(wildcard include/config/arm/l1/cache/shift.h) \
    $(wildcard include/config/aeabi.h) \
  include/linux/kernel.h \
    $(wildcard include/config/preempt/voluntary.h) \
    $(wildcard include/config/debug/spinlock/sleep.h) \
    $(wildcard include/config/prove/locking.h) \
    $(wildcard include/config/printk.h) \
    $(wildcard include/config/dynamic/debug.h) \
    $(wildcard include/config/ring/buffer.h) \
    $(wildcard include/config/tracing.h) \
    $(wildcard include/config/numa.h) \
    $(wildcard include/config/ftrace/mcount/record.h) \
  /home/andy/CodeSourcery/Sourcery_G++_Lite/bin/../lib/gcc/arm-none-linux-gnueabi/4.3.3/include/stdarg.h \
  include/linux/log2.h \
    $(wildcard include/config/arch/has/ilog2/u32.h) \
    $(wildcard include/config/arch/has/ilog2/u64.h) \
  include/linux/dynamic_debug.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/byteorder.h \
  include/linux/byteorder/little_endian.h \
  include/linux/swab.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/swab.h \
  include/linux/byteorder/generic.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/bug.h \
    $(wildcard include/config/bug.h) \
    $(wildcard include/config/debug/bugverbose.h) \
  include/asm-generic/bug.h \
    $(wildcard include/config/generic/bug.h) \
    $(wildcard include/config/generic/bug/relative/pointers.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/div64.h \
  include/linux/stringify.h \
  include/linux/bottom_half.h \
  include/linux/spinlock_types.h \
  include/linux/spinlock_types_up.h \
  include/linux/lockdep.h \
    $(wildcard include/config/lockdep.h) \
    $(wildcard include/config/lock/stat.h) \
    $(wildcard include/config/generic/hardirqs.h) \
  include/linux/rwlock_types.h \
  include/linux/spinlock_up.h \
  include/linux/rwlock.h \
  include/linux/spinlock_api_up.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/atomic.h \
  include/asm-generic/atomic-long.h \
  include/linux/semaphore.h \
  include/linux/sem.h \
    $(wildcard include/config/sysvipc.h) \
  include/linux/ipc.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/ipcbuf.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/sembuf.h \
  include/linux/rcupdate.h \
    $(wildcard include/config/tree/rcu.h) \
    $(wildcard include/config/tree/preempt/rcu.h) \
    $(wildcard include/config/tiny/rcu.h) \
  include/linux/cache.h \
    $(wildcard include/config/arch/has/cache/line/size.h) \
  include/linux/threads.h \
    $(wildcard include/config/nr/cpus.h) \
    $(wildcard include/config/base/small.h) \
  include/linux/cpumask.h \
    $(wildcard include/config/cpumask/offstack.h) \
    $(wildcard include/config/hotplug/cpu.h) \
    $(wildcard include/config/debug/per/cpu/maps.h) \
    $(wildcard include/config/disable/obsolete/cpumask/functions.h) \
  include/linux/bitmap.h \
  include/linux/string.h \
    $(wildcard include/config/binary/printf.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/string.h \
  include/linux/seqlock.h \
  include/linux/completion.h \
  include/linux/wait.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/current.h \
  include/linux/rcutree.h \
    $(wildcard include/config/no/hz.h) \
  include/linux/sched.h \
    $(wildcard include/config/sched/debug.h) \
    $(wildcard include/config/detect/softlockup.h) \
    $(wildcard include/config/detect/hung/task.h) \
    $(wildcard include/config/core/dump/default/elf/headers.h) \
    $(wildcard include/config/virt/cpu/accounting.h) \
    $(wildcard include/config/bsd/process/acct.h) \
    $(wildcard include/config/taskstats.h) \
    $(wildcard include/config/audit.h) \
    $(wildcard include/config/inotify/user.h) \
    $(wildcard include/config/epoll.h) \
    $(wildcard include/config/posix/mqueue.h) \
    $(wildcard include/config/keys.h) \
    $(wildcard include/config/user/sched.h) \
    $(wildcard include/config/sysfs.h) \
    $(wildcard include/config/perf/events.h) \
    $(wildcard include/config/schedstats.h) \
    $(wildcard include/config/task/delay/acct.h) \
    $(wildcard include/config/fair/group/sched.h) \
    $(wildcard include/config/rt/group/sched.h) \
    $(wildcard include/config/blk/dev/io/trace.h) \
    $(wildcard include/config/cc/stackprotector.h) \
    $(wildcard include/config/auditsyscall.h) \
    $(wildcard include/config/rt/mutexes.h) \
    $(wildcard include/config/debug/mutexes.h) \
    $(wildcard include/config/task/xacct.h) \
    $(wildcard include/config/cpusets.h) \
    $(wildcard include/config/cgroups.h) \
    $(wildcard include/config/futex.h) \
    $(wildcard include/config/fault/injection.h) \
    $(wildcard include/config/latencytop.h) \
    $(wildcard include/config/function/graph/tracer.h) \
    $(wildcard include/config/cgroup/mem/res/ctlr.h) \
    $(wildcard include/config/have/unstable/sched/clock.h) \
    $(wildcard include/config/stack/growsup.h) \
    $(wildcard include/config/debug/stack/usage.h) \
    $(wildcard include/config/group/sched.h) \
    $(wildcard include/config/mm/owner.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/param.h \
    $(wildcard include/config/hz.h) \
  include/linux/capability.h \
  include/linux/timex.h \
  include/linux/time.h \
    $(wildcard include/config/arch/uses/gettimeoffset.h) \
  include/linux/math64.h \
  include/linux/param.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/timex.h \
  arch/arm/mach-davinci/include/mach/timex.h \
  include/linux/jiffies.h \
  include/linux/rbtree.h \
  include/linux/errno.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/errno.h \
  include/asm-generic/errno.h \
  include/asm-generic/errno-base.h \
  include/linux/nodemask.h \
    $(wildcard include/config/highmem.h) \
  include/linux/numa.h \
    $(wildcard include/config/nodes/shift.h) \
  include/linux/mm_types.h \
    $(wildcard include/config/split/ptlock/cpus.h) \
    $(wildcard include/config/want/page/debug/flags.h) \
    $(wildcard include/config/kmemcheck.h) \
    $(wildcard include/config/aio.h) \
    $(wildcard include/config/proc/fs.h) \
    $(wildcard include/config/mmu/notifier.h) \
  include/linux/auxvec.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/auxvec.h \
  include/linux/prio_tree.h \
  include/linux/rwsem.h \
    $(wildcard include/config/rwsem/generic/spinlock.h) \
  include/linux/rwsem-spinlock.h \
  include/linux/page-debug-flags.h \
    $(wildcard include/config/page/poisoning.h) \
    $(wildcard include/config/page/debug/something/else.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/page.h \
    $(wildcard include/config/cpu/copy/v3.h) \
    $(wildcard include/config/cpu/copy/v4wt.h) \
    $(wildcard include/config/cpu/copy/v4wb.h) \
    $(wildcard include/config/cpu/copy/feroceon.h) \
    $(wildcard include/config/cpu/copy/fa.h) \
    $(wildcard include/config/cpu/xscale.h) \
    $(wildcard include/config/cpu/copy/v6.h) \
    $(wildcard include/config/sparsemem.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/glue.h \
    $(wildcard include/config/cpu/arm610.h) \
    $(wildcard include/config/cpu/arm710.h) \
    $(wildcard include/config/cpu/abrt/lv4t.h) \
    $(wildcard include/config/cpu/abrt/ev4.h) \
    $(wildcard include/config/cpu/abrt/ev4t.h) \
    $(wildcard include/config/cpu/abrt/ev5tj.h) \
    $(wildcard include/config/cpu/abrt/ev5t.h) \
    $(wildcard include/config/cpu/abrt/ev6.h) \
    $(wildcard include/config/cpu/abrt/ev7.h) \
    $(wildcard include/config/cpu/pabrt/legacy.h) \
    $(wildcard include/config/cpu/pabrt/v6.h) \
    $(wildcard include/config/cpu/pabrt/v7.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/memory.h \
    $(wildcard include/config/page/offset.h) \
    $(wildcard include/config/thumb2/kernel.h) \
    $(wildcard include/config/dram/size.h) \
    $(wildcard include/config/dram/base.h) \
    $(wildcard include/config/zone/dma.h) \
    $(wildcard include/config/discontigmem.h) \
  include/linux/const.h \
  arch/arm/mach-davinci/include/mach/memory.h \
    $(wildcard include/config/arch/davinci/da8xx.h) \
    $(wildcard include/config/arch/davinci/dmx.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/sizes.h \
  include/asm-generic/memory_model.h \
    $(wildcard include/config/flatmem.h) \
    $(wildcard include/config/sparsemem/vmemmap.h) \
  include/asm-generic/getorder.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/mmu.h \
    $(wildcard include/config/cpu/has/asid.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/cputime.h \
  include/asm-generic/cputime.h \
  include/linux/smp.h \
    $(wildcard include/config/use/generic/smp/helpers.h) \
  include/linux/signal.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/signal.h \
  include/asm-generic/signal-defs.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/sigcontext.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/siginfo.h \
  include/asm-generic/siginfo.h \
  include/linux/path.h \
  include/linux/pid.h \
  include/linux/percpu.h \
    $(wildcard include/config/modules.h) \
    $(wildcard include/config/need/per/cpu/embed/first/chunk.h) \
    $(wildcard include/config/need/per/cpu/page/first/chunk.h) \
    $(wildcard include/config/have/setup/per/cpu/area.h) \
  include/linux/slab.h \
    $(wildcard include/config/slab/debug.h) \
    $(wildcard include/config/debug/objects.h) \
    $(wildcard include/config/slub.h) \
    $(wildcard include/config/slob.h) \
    $(wildcard include/config/debug/slab.h) \
  include/linux/gfp.h \
    $(wildcard include/config/zone/dma32.h) \
    $(wildcard include/config/debug/vm.h) \
  include/linux/mmzone.h \
    $(wildcard include/config/force/max/zoneorder.h) \
    $(wildcard include/config/memory/hotplug.h) \
    $(wildcard include/config/arch/populates/node/map.h) \
    $(wildcard include/config/flat/node/mem/map.h) \
    $(wildcard include/config/have/memory/present.h) \
    $(wildcard include/config/need/node/memmap/size.h) \
    $(wildcard include/config/need/multiple/nodes.h) \
    $(wildcard include/config/have/arch/early/pfn/to/nid.h) \
    $(wildcard include/config/sparsemem/extreme.h) \
    $(wildcard include/config/nodes/span/other/nodes.h) \
    $(wildcard include/config/holes/in/zone.h) \
    $(wildcard include/config/arch/has/holes/memorymodel.h) \
  include/linux/init.h \
    $(wildcard include/config/hotplug.h) \
  include/linux/pageblock-flags.h \
    $(wildcard include/config/hugetlb/page.h) \
    $(wildcard include/config/hugetlb/page/size/variable.h) \
  include/generated/bounds.h \
  include/linux/memory_hotplug.h \
    $(wildcard include/config/have/arch/nodedata/extension.h) \
    $(wildcard include/config/memory/hotremove.h) \
  include/linux/notifier.h \
  include/linux/mutex.h \
  include/linux/srcu.h \
  include/linux/topology.h \
    $(wildcard include/config/sched/smt.h) \
    $(wildcard include/config/sched/mc.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/topology.h \
  include/asm-generic/topology.h \
  include/linux/mmdebug.h \
    $(wildcard include/config/debug/virtual.h) \
  include/linux/slub_def.h \
    $(wildcard include/config/slub/stats.h) \
    $(wildcard include/config/slub/debug.h) \
  include/linux/workqueue.h \
    $(wildcard include/config/debug/objects/work.h) \
  include/linux/timer.h \
    $(wildcard include/config/timer/stats.h) \
    $(wildcard include/config/debug/objects/timers.h) \
  include/linux/ktime.h \
    $(wildcard include/config/ktime/scalar.h) \
  include/linux/debugobjects.h \
    $(wildcard include/config/debug/objects/free.h) \
  include/linux/kobject.h \
  include/linux/sysfs.h \
  include/linux/kref.h \
  include/linux/kmemtrace.h \
    $(wildcard include/config/kmemtrace.h) \
  include/trace/events/kmem.h \
  include/linux/tracepoint.h \
    $(wildcard include/config/tracepoints.h) \
  include/trace/define_trace.h \
    $(wildcard include/config/event/tracing.h) \
  include/linux/kmemleak.h \
    $(wildcard include/config/debug/kmemleak.h) \
  include/linux/pfn.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/percpu.h \
  include/asm-generic/percpu.h \
  include/linux/percpu-defs.h \
    $(wildcard include/config/debug/force/weak/per/cpu.h) \
  include/linux/proportions.h \
  include/linux/percpu_counter.h \
  include/linux/seccomp.h \
    $(wildcard include/config/seccomp.h) \
  include/linux/rculist.h \
  include/linux/rtmutex.h \
    $(wildcard include/config/debug/rt/mutexes.h) \
  include/linux/plist.h \
    $(wildcard include/config/debug/pi/list.h) \
  include/linux/resource.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/resource.h \
  include/asm-generic/resource.h \
  include/linux/hrtimer.h \
    $(wildcard include/config/high/res/timers.h) \
  include/linux/task_io_accounting.h \
    $(wildcard include/config/task/io/accounting.h) \
  include/linux/latencytop.h \
  include/linux/cred.h \
    $(wildcard include/config/debug/credentials.h) \
    $(wildcard include/config/security.h) \
  include/linux/key.h \
    $(wildcard include/config/sysctl.h) \
  include/linux/sysctl.h \
  include/linux/selinux.h \
    $(wildcard include/config/security/selinux.h) \
  include/linux/aio.h \
  include/linux/aio_abi.h \
  include/linux/uio.h \
  include/linux/netdevice.h \
    $(wildcard include/config/dcb.h) \
    $(wildcard include/config/wlan/80211.h) \
    $(wildcard include/config/ax25.h) \
    $(wildcard include/config/mac80211/mesh.h) \
    $(wildcard include/config/tr.h) \
    $(wildcard include/config/net/ipip.h) \
    $(wildcard include/config/net/ipgre.h) \
    $(wildcard include/config/ipv6/sit.h) \
    $(wildcard include/config/ipv6/tunnel.h) \
    $(wildcard include/config/netpoll.h) \
    $(wildcard include/config/net/poll/controller.h) \
    $(wildcard include/config/fcoe.h) \
    $(wildcard include/config/wireless/ext.h) \
    $(wildcard include/config/net/dsa.h) \
    $(wildcard include/config/net/ns.h) \
    $(wildcard include/config/net/dsa/tag/dsa.h) \
    $(wildcard include/config/net/dsa/tag/trailer.h) \
    $(wildcard include/config/netpoll/trap.h) \
  include/linux/if.h \
  include/linux/socket.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/socket.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/sockios.h \
  include/linux/sockios.h \
  include/linux/hdlc/ioctl.h \
  include/linux/if_ether.h \
  include/linux/skbuff.h \
    $(wildcard include/config/nf/conntrack.h) \
    $(wildcard include/config/bridge/netfilter.h) \
    $(wildcard include/config/has/dma.h) \
    $(wildcard include/config/xfrm.h) \
    $(wildcard include/config/net/sched.h) \
    $(wildcard include/config/net/cls/act.h) \
    $(wildcard include/config/ipv6/ndisc/nodetype.h) \
    $(wildcard include/config/net/dma.h) \
    $(wildcard include/config/network/secmark.h) \
  include/linux/kmemcheck.h \
  include/linux/net.h \
  include/linux/random.h \
  include/linux/ioctl.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/ioctl.h \
  include/asm-generic/ioctl.h \
  include/linux/irqnr.h \
  include/linux/fcntl.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/fcntl.h \
  include/asm-generic/fcntl.h \
  include/linux/ratelimit.h \
  include/linux/textsearch.h \
  include/linux/module.h \
    $(wildcard include/config/symbol/prefix.h) \
    $(wildcard include/config/modversions.h) \
    $(wildcard include/config/unused/symbols.h) \
    $(wildcard include/config/kallsyms.h) \
    $(wildcard include/config/module/unload.h) \
    $(wildcard include/config/constructors.h) \
  include/linux/stat.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/stat.h \
  include/linux/kmod.h \
  include/linux/elf.h \
  include/linux/elf-em.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/elf.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/user.h \
  include/linux/moduleparam.h \
    $(wildcard include/config/alpha.h) \
    $(wildcard include/config/ia64.h) \
    $(wildcard include/config/ppc64.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/local.h \
  include/asm-generic/local.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/module.h \
    $(wildcard include/config/arm/unwind.h) \
  include/trace/events/module.h \
  include/linux/err.h \
  include/net/checksum.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/uaccess.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/unified.h \
    $(wildcard include/config/arm/asm/unified.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/checksum.h \
  include/linux/in6.h \
  include/linux/dmaengine.h \
    $(wildcard include/config/dma/engine.h) \
    $(wildcard include/config/async/tx/dma.h) \
    $(wildcard include/config/async/tx/disable/channel/switch.h) \
  include/linux/device.h \
    $(wildcard include/config/debug/devres.h) \
    $(wildcard include/config/devtmpfs.h) \
  include/linux/ioport.h \
  include/linux/klist.h \
  include/linux/pm.h \
    $(wildcard include/config/pm/sleep.h) \
    $(wildcard include/config/pm/runtime.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/device.h \
    $(wildcard include/config/dmabounce.h) \
  include/linux/pm_wakeup.h \
  include/linux/dma-mapping.h \
    $(wildcard include/config/have/dma/attrs.h) \
  include/linux/dma-attrs.h \
  include/linux/bug.h \
  include/linux/scatterlist.h \
    $(wildcard include/config/debug/sg.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/scatterlist.h \
  include/linux/mm.h \
    $(wildcard include/config/ksm.h) \
    $(wildcard include/config/debug/pagealloc.h) \
    $(wildcard include/config/hibernation.h) \
  include/linux/debug_locks.h \
    $(wildcard include/config/debug/locking/api/selftests.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/pgtable.h \
    $(wildcard include/config/highpte.h) \
  include/asm-generic/4level-fixup.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/proc-fns.h \
    $(wildcard include/config/cpu/arm7tdmi.h) \
    $(wildcard include/config/cpu/arm720t.h) \
    $(wildcard include/config/cpu/arm740t.h) \
    $(wildcard include/config/cpu/arm9tdmi.h) \
    $(wildcard include/config/cpu/arm920t.h) \
    $(wildcard include/config/cpu/arm922t.h) \
    $(wildcard include/config/cpu/arm925t.h) \
    $(wildcard include/config/cpu/arm926t.h) \
    $(wildcard include/config/cpu/arm940t.h) \
    $(wildcard include/config/cpu/arm946e.h) \
    $(wildcard include/config/cpu/arm1020.h) \
    $(wildcard include/config/cpu/arm1020e.h) \
    $(wildcard include/config/cpu/arm1022.h) \
    $(wildcard include/config/cpu/arm1026.h) \
    $(wildcard include/config/cpu/mohawk.h) \
    $(wildcard include/config/cpu/feroceon.h) \
    $(wildcard include/config/cpu/v6.h) \
    $(wildcard include/config/cpu/v7.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/cpu-single.h \
  arch/arm/mach-davinci/include/mach/vmalloc.h \
  arch/arm/mach-davinci/include/mach/hardware.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/pgtable-hwdef.h \
  include/asm-generic/pgtable.h \
  include/linux/page-flags.h \
    $(wildcard include/config/pageflags/extended.h) \
    $(wildcard include/config/arch/uses/pg/uncached.h) \
    $(wildcard include/config/memory/failure.h) \
    $(wildcard include/config/swap.h) \
    $(wildcard include/config/s390.h) \
  include/linux/vmstat.h \
    $(wildcard include/config/vm/event/counters.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/io.h \
  arch/arm/mach-davinci/include/mach/io.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/dma-mapping.h \
  include/asm-generic/dma-coherent.h \
    $(wildcard include/config/have/generic/dma/coherent.h) \
  include/linux/if_packet.h \
  include/linux/delay.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/delay.h \
  include/linux/ethtool.h \
  include/net/net_namespace.h \
    $(wildcard include/config/ipv6.h) \
    $(wildcard include/config/ip/dccp.h) \
    $(wildcard include/config/netfilter.h) \
    $(wildcard include/config/wext/core.h) \
    $(wildcard include/config/net.h) \
  include/net/netns/core.h \
  include/net/netns/mib.h \
    $(wildcard include/config/xfrm/statistics.h) \
  include/net/snmp.h \
  include/linux/snmp.h \
  include/net/netns/unix.h \
  include/net/netns/packet.h \
  include/net/netns/ipv4.h \
    $(wildcard include/config/ip/multiple/tables.h) \
    $(wildcard include/config/ip/mroute.h) \
    $(wildcard include/config/ip/pimsm/v1.h) \
    $(wildcard include/config/ip/pimsm/v2.h) \
  include/net/inet_frag.h \
  include/net/netns/ipv6.h \
    $(wildcard include/config/ipv6/multiple/tables.h) \
    $(wildcard include/config/ipv6/mroute.h) \
    $(wildcard include/config/ipv6/pimsm/v2.h) \
  include/net/dst_ops.h \
  include/net/netns/dccp.h \
  include/net/netns/x_tables.h \
    $(wildcard include/config/bridge/nf/ebtables.h) \
  include/linux/netfilter.h \
    $(wildcard include/config/netfilter/debug.h) \
    $(wildcard include/config/nf/nat/needed.h) \
  include/linux/in.h \
  include/net/flow.h \
  include/linux/proc_fs.h \
    $(wildcard include/config/proc/devicetree.h) \
    $(wildcard include/config/proc/kcore.h) \
  include/linux/fs.h \
    $(wildcard include/config/dnotify.h) \
    $(wildcard include/config/quota.h) \
    $(wildcard include/config/fsnotify.h) \
    $(wildcard include/config/inotify.h) \
    $(wildcard include/config/fs/posix/acl.h) \
    $(wildcard include/config/debug/writecount.h) \
    $(wildcard include/config/file/locking.h) \
    $(wildcard include/config/block.h) \
    $(wildcard include/config/fs/xip.h) \
    $(wildcard include/config/migration.h) \
  include/linux/limits.h \
  include/linux/kdev_t.h \
  include/linux/dcache.h \
  include/linux/radix-tree.h \
  include/linux/fiemap.h \
  include/linux/quota.h \
    $(wildcard include/config/quota/netlink/interface.h) \
  include/linux/dqblk_xfs.h \
  include/linux/dqblk_v1.h \
  include/linux/dqblk_v2.h \
  include/linux/dqblk_qtree.h \
  include/linux/nfs_fs_i.h \
  include/linux/nfs.h \
  include/linux/sunrpc/msg_prot.h \
  include/linux/inet.h \
  include/linux/magic.h \
  include/net/netns/xfrm.h \
  include/linux/xfrm.h \
  include/linux/seq_file_net.h \
  include/linux/seq_file.h \
  include/net/dsa.h \
  include/linux/interrupt.h \
    $(wildcard include/config/generic/irq/probe.h) \
  include/linux/irqreturn.h \
  include/linux/hardirq.h \
  include/linux/smp_lock.h \
    $(wildcard include/config/lock/kernel.h) \
  include/linux/ftrace_irq.h \
    $(wildcard include/config/ftrace/nmi/enter.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/hardirq.h \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/irq.h \
  arch/arm/mach-davinci/include/mach/irqs.h \
  include/linux/irq_cpustat.h \
  include/linux/etherdevice.h \
    $(wildcard include/config/have/efficient/unaligned/access.h) \
  /home/andy/git/lejos/kernel/linux-03.20.00.13/arch/arm/include/asm/unaligned.h \
  include/linux/unaligned/le_byteshift.h \
  include/linux/unaligned/be_byteshift.h \
  include/linux/unaligned/generic.h \
  include/net/iw_handler.h \
    $(wildcard include/config/wext/priv.h) \
  include/linux/wireless.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_byteorder.h \
    $(wildcard include/config/platform/mstar389.h) \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/byteorder/little_endian.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/drv_types.h \
    $(wildcard include/config/1t1r.h) \
    $(wildcard include/config/2t2r.h) \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/wlan_bssdef.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtl8192c_spec.h \
    $(wildcard include/config/usedk.h) \
    $(wildcard include/config/no/usedk.h) \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/basic_types.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/drv_types_linux.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_ht.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/wifi.h \
    $(wildcard include/config/rtl8712fw.h) \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_cmd.h \
    $(wildcard include/config/rtl8711fw.h) \
    $(wildcard include/config/event/thread/mode.h) \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_rf.h \
    $(wildcard include/config/.h) \
    $(wildcard include/config/1t.h) \
    $(wildcard include/config/2t.h) \
    $(wildcard include/config/1r.h) \
    $(wildcard include/config/2r.h) \
    $(wildcard include/config/1t2r.h) \
    $(wildcard include/config/turbo.h) \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/ieee80211.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtl8192c_cmd.h \
    $(wildcard include/config/eid.h) \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_xmit.h \
    $(wildcard include/config/rtl8712/tcp/csum/offload/tx.h) \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/xmit_osdep.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtl8192c_xmit.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_recv.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtl8192c_recv.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/hal_init.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtl8192c_hal.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/Hal8192CPhyReg.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/Hal8192CPhyCfg.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtl8192c_dm.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/Hal8192CUHWImg.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_qos.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_security.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_pwrctrl.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_io.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/osdep_intf.h \
  include/linux/usb.h \
    $(wildcard include/config/usb/devicefs.h) \
    $(wildcard include/config/usb/mon.h) \
    $(wildcard include/config/usb/device/class.h) \
    $(wildcard include/config/usb/suspend.h) \
  include/linux/mod_devicetable.h \
  include/linux/usb/ch9.h \
    $(wildcard include/config/size.h) \
    $(wildcard include/config/att/one.h) \
    $(wildcard include/config/att/selfpower.h) \
    $(wildcard include/config/att/wakeup.h) \
    $(wildcard include/config/att/battery.h) \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_eeprom.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/sta_info.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/wifi.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_mlme.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_mp.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_debug.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_event.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtl8192c_event.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_led.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/rtw_mlme_ext.h \
  /home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/include/usb_hal.h \

/home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/hal/hal_init.o: $(deps_/home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/hal/hal_init.o)

$(deps_/home/andy/git/lejos/modules/net/rtl8192CU_linux_v2.0.939.20100726/hal/hal_init.o):
