# TOE-rm-radar
robomaster雷达站

## __radar_detector__ 雷达检测主模块
- RadarIdentificationNode 检测目标信息和在图像坐标系中的位置
    - 数据发布：
- RadarOrientationNode 定位目标在世界坐标系中的位置
    - 数据接收:
        - 类型Identification.msg _目标在图像坐标系中的位置_
        - 类型Status.msg _目标在图像坐标系中的位置_
        - 类型Keyboard.msg _键盘输入_
    - 数据发布：
        - 类型Tf _各个兵种的地图坐标_
        - 类型CalibrationUi.msg _标定框坐标_
        - 类型CalibrationTf.msg _自定义检测框坐标_
        
