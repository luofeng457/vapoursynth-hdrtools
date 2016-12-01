vapoursynth-hdrtools
===================

# About
HDRTools as a vapoursynth plugin.

# Build

  clone code:

  ```shell
  $ git clone https://github.com/ArthurChiao/vapoursynth-hdrtools.git
  ```

  compile HDRTools:

  ```shell
  $ cd vapoursynth-hdrtools
  $ make
  ```

  compile plugin `vapoursynth-hdrtools`:

  ```shell
  $ cd gen-pluin
  $ make

  $ ls *.so
  -rwxrwxr-x 1 arthurchiao arthurchiao 1.9M Dec  1 20:44 libhdrconv.so*
  ```
  the `libhdrconv.so` is just the plugin.

# Test

  `test-vapoursynth-hdrtools.vpy`:

  ```python
  import vapoursynth as vs

  core = vs.get_core()
  core.std.LoadPlugin("/home/arthurchiao/libhdrconv.so")

  print(core.list_functions())
  ```

  The script will list all the functions in all the plugins it loaded, execute
  it with `vspipe`:

  ```shell
  $ vspipe test-vapoursynth-hdrtools.vpy - > result.txt

  # check if our plugin has been successfully loaded
  $ cat result.txt | grep -B 2 hdrconv
  name: Format converter, v0.0.1
  namespace: cn.edu.sjtu.medialab
  identifier: hdrconv
  ```

# Docker Image

**TODO**

# License
**I'm not sure if this is ok, write to me if I'm infringing**

1. **HDRTools**: same license as HDRTools (BSD)

  All HDRTools code.

1. **vapoursynth**: same as vapoursynth (LGPL)

   `VapourSynth.h`, `VSHelper.h`, `VSScript.h`

1. **vapoursynth-hdrtools**

  * new code: BSD license
  * library: BSD/LGPL dual-licese

