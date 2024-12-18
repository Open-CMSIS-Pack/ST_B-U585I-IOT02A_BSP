# Continuous Integration (CI) Test for Board Support Pack (BSP)

Content of `.ci` Directory      | Description
:-------------------------------|:-----------------
`./Layers/GCC/Default`          | Default Board layer for the GCC compiler.
`./Layers/GCC/USBD_WiFi_Sensors`| Board Layer supporting USB Device, Audio, Environment and Motion Sensors for the GCC compiler.
`./Layers/GCC/USBH_WiFi_Sensors`| Board Layer supporting USB Host,   Audio, Environment and Motion Sensors for the GCC compiler.
`vcpkg-configuration.json`      | Tool setup for the CI test

The [GitHub Actions](https://github.com/Open-CMSIS-Pack/ST_B-U585I-IOT02A_BSP/tree/main/README.md#github-actions) in the directory [`.github/workflows`](https://github.com/Open-CMSIS-Pack/ST_B-U585I-IOT02A_BSP/tree/main/.github/workflows) are the scripts for the CI tests. These scripts contain detailed comments about each step that is executed.

For Reference Applications the board layers are copied during test execution. As board layers are configured by default for the AC6 compiler, different board layers for the GCC compiler are contained in this `.ci` directory.

Some Reference Applications require patches as not every variant of the csolution project can be targeted to this board. These patches are also contained in this `.ci` directory.

The output of the build test is stored as artifacts and can be downloaded to the evaluation board for execution.
