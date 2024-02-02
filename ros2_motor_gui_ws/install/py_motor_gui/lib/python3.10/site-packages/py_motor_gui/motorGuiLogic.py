
import rclpy

from PyQt5           import *
from .motorGuiNode   import WindowSubscriber
from .motorGuiWindow import *

class MainWindow(QtWidgets.QMainWindow, Ui_Dialog):

    def __init__(self, *args, **kwargs):
        QtWidgets.QMainWindow.__init__(self, *args, **kwargs)
        self.setupUi(self)
        self.initUI()

        self.subscriber = WindowSubscriber()

        self.window_subscriber = WindowSubscriber
    
    def initUI(self):

        self.positionAxialNeck_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialNeck_slider, self.velocityAxialNeck_slider, self.positionValueAxialNeck_label, self.velocityValueAxialNeck_label, 'AxialNeck', 'neck'))
        self.velocityAxialNeck_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialNeck_slider, self.velocityAxialNeck_slider, self.positionValueAxialNeck_label, self.velocityValueAxialNeck_label, 'AxialNeck', 'neck'))
        self.positionFrontalNeck_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalNeck_slider, self.velocityFrontalNeck_slider, self.positionValueFrontalNeck_label, self.velocityValueFrontalNeck_label, 'FrontalNeck', 'neck'))
        self.velocityFrontalNeck_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalNeck_slider, self.velocityFrontalNeck_slider, self.positionValueFrontalNeck_label, self.velocityValueFrontalNeck_label, 'FrontalNeck', 'neck'))

        self.positionFrontalRightShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightShoulder_slider, self.velocityFrontalRightShoulder_slider, self.positionValueFrontalRightShoulder_label, self.velocityValueFrontalRightShoulder_label, 'FrontalRightShoulder', 'rightArm'))
        self.velocityFrontalRightShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightShoulder_slider, self.velocityFrontalRightShoulder_slider, self.positionValueFrontalRightShoulder_label, self.velocityValueFrontalRightShoulder_label, 'FrontalRightShoulder', 'rightArm'))
        self.positionSagittalRightShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalRightShoulder_slider, self.velocitySagittalRightShoulder_slider, self.positionValueSagittalRightShoulder_label, self.velocityValueSagittalRightShoulder_label, 'SagittalRightShoulder', 'rightArm'))
        self.velocitySagittalRightShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalRightShoulder_slider, self.velocitySagittalRightShoulder_slider, self.positionValueSagittalRightShoulder_label, self.velocityValueSagittalRightShoulder_label, 'SagittalRightShoulder', 'rightArm'))
        self.positionAxialRightShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialRightShoulder_slider, self.velocityAxialRightShoulder_slider, self.positionValueAxialRightShoulder_label, self.velocityValueAxialRightShoulder_label, 'AxialRightShoulder', 'rightArm'))
        self.velocityAxialRightShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialRightShoulder_slider, self.velocityAxialRightShoulder_slider, self.positionValueAxialRightShoulder_label, self.velocityValueAxialRightShoulder_label, 'AxialRightShoulder', 'rightArm'))
        self.positionFrontalRightElbow_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightElbow_slider, self.velocityFrontalRightElbow_slider, self.positionValueFrontalRightElbow_label, self.velocityValueFrontalRightElbow_label, 'FrontalRightElbow', 'rightArm'))
        self.velocityFrontalRightElbow_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightElbow_slider, self.velocityFrontalRightElbow_slider, self.positionValueFrontalRightElbow_label, self.velocityValueFrontalRightElbow_label, 'FrontalRightElbow', 'rightArm'))
        self.positionAxialRightWrist_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialRightWrist_slider, self.velocityAxialRightWrist_slider, self.positionValueAxialRightWrist_label, self.velocityValueAxialRightWrist_label, 'AxialRightWrist', 'rightArm'))
        self.velocityAxialRightWrist_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialRightWrist_slider, self.velocityAxialRightWrist_slider, self.positionValueAxialRightWrist_label, self.velocityValueAxialRightWrist_label, 'AxialRightWrist', 'rightArm'))
        self.positionFrontalRightWrist_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightWrist_slider, self.velocityFrontalRightWrist_slider, self.positionValueFrontalRightWrist_label, self.velocityValueFrontalRightWrist_label, 'FrontalRightWrist', 'rightArm'))
        self.velocityFrontalRightWrist_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightWrist_slider, self.velocityFrontalRightWrist_slider, self.positionValueFrontalRightWrist_label, self.velocityValueFrontalRightWrist_label, 'FrontalRightWrist', 'rightArm'))

        self.positionFrontalLeftShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftShoulder_slider, self.velocityFrontalLeftShoulder_slider, self.positionValueFrontalLeftShoulder_label, self.velocityValueFrontalLeftShoulder_label, 'FrontalLeftShoulder', 'leftArm'))
        self.velocityFrontalLeftShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftShoulder_slider, self.velocityFrontalLeftShoulder_slider, self.positionValueFrontalLeftShoulder_label, self.velocityValueFrontalLeftShoulder_label, 'FrontalLeftShoulder', 'leftArm'))
        self.positionSagittalLeftShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalLeftShoulder_slider, self.velocitySagittalLeftShoulder_slider, self.positionValueSagittalLeftShoulder_label, self.velocityValueSagittalLeftShoulder_label, 'SagittalLeftShoulder', 'leftArm'))
        self.velocitySagittalLeftShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalLeftShoulder_slider, self.velocitySagittalLeftShoulder_slider, self.positionValueSagittalLeftShoulder_label, self.velocityValueSagittalLeftShoulder_label, 'SagittalLeftShoulder', 'leftArm'))
        self.positionAxialLeftShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialLeftShoulder_slider, self.velocityAxialLeftShoulder_slider, self.positionValueAxialLeftShoulder_label, self.velocityValueAxialLeftShoulder_label, 'AxialLeftShoulder', 'leftArm'))
        self.velocityAxialLeftShoulder_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialLeftShoulder_slider, self.velocityAxialLeftShoulder_slider, self.positionValueAxialLeftShoulder_label, self.velocityValueAxialLeftShoulder_label, 'AxialLeftShoulder', 'leftArm'))
        self.positionFrontalLeftElbow_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftElbow_slider, self.velocityFrontalLeftElbow_slider, self.positionValueFrontalLeftElbow_label, self.velocityValueFrontalLeftElbow_label, 'FrontalLeftElbow', 'leftArm'))
        self.velocityFrontalLeftElbow_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftElbow_slider, self.velocityFrontalLeftElbow_slider, self.positionValueFrontalLeftElbow_label, self.velocityValueFrontalLeftElbow_label, 'FrontalLeftElbow', 'leftArm'))
        self.positionAxialLeftWrist_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialLeftWrist_slider, self.velocityAxialLeftWrist_slider, self.positionValueAxialLeftWrist_label, self.velocityValueAxialLeftWrist_label, 'AxialLeftWrist', 'leftArm'))
        self.velocityAxialLeftWrist_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialLeftWrist_slider, self.velocityAxialLeftWrist_slider, self.positionValueAxialLeftWrist_label, self.velocityValueAxialLeftWrist_label, 'AxialLeftWrist', 'leftArm'))
        self.positionFrontalLeftWrist_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftWrist_slider, self.velocityFrontalLeftWrist_slider, self.positionValueFrontalLeftWrist_label, self.velocityValueFrontalLeftWrist_label, 'FrontalLeftWrist', 'leftArm'))
        self.velocityFrontalLeftWrist_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftWrist_slider, self.velocityFrontalLeftWrist_slider, self.positionValueFrontalLeftWrist_label, self.velocityValueFrontalLeftWrist_label, 'FrontalLeftWrist', 'leftArm'))
     
        self.positionAxialTrunk_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialTrunk_slider, self.velocityAxialTrunk_slider, self.positionValueAxialTrunk_label, self.velocityValueAxialTrunk_label, 'AxialTrunk', 'trunk'))
        self.velocityAxialTrunk_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialTrunk_slider, self.velocityAxialTrunk_slider, self.positionValueAxialTrunk_label, self.velocityValueAxialTrunk_label, 'AxialTrunk', 'trunk'))
        self.positionFrontalTrunk_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalTrunk_slider, self.velocityFrontalTrunk_slider, self.positionValueFrontalTrunk_label, self.velocityValueFrontalTrunk_label, 'FrontalTrunk', 'trunk'))
        self.velocityFrontalTrunk_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalTrunk_slider, self.velocityFrontalTrunk_slider, self.positionValueFrontalTrunk_label, self.velocityValueFrontalTrunk_label, 'FrontalTrunk', 'trunk'))

        self.positionAxialRightHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialRightHip_slider, self.velocityAxialRightHip_slider, self.positionValueAxialRightHip_label, self.velocityValueAxialRightHip_label, 'AxialRightHip', 'rightLeg'))
        self.velocityAxialRightHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialRightHip_slider, self.velocityAxialRightHip_slider, self.positionValueAxialRightHip_label, self.velocityValueAxialRightHip_label, 'AxialRightHip', 'rightLeg'))
        self.positionSagittalRightHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalRightHip_slider, self.velocitySagittalRightHip_slider, self.positionValueSagittalRightHip_label, self.velocityValueSagittalRightHip_label,'SagittalRightHip', 'rightLeg'))
        self.velocitySagittalRightHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalRightHip_slider, self.velocitySagittalRightHip_slider, self.positionValueSagittalRightHip_label, self.velocityValueSagittalRightHip_label, 'SagittalRightHip', 'rightLeg'))
        self.positionFrontalRightHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightHip_slider, self.velocityFrontalRightHip_slider, self.positionValueFrontalRightHip_label, self.velocityValueFrontalRightHip_label, 'FrontalRightHip', 'rightLeg'))
        self.velocityFrontalRightHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightHip_slider, self.velocityFrontalRightHip_slider, self.positionValueFrontalRightHip_label, self.velocityValueFrontalRightHip_label, 'FrontalRightHip', 'rightLeg'))
        self.positionFrontalRightKnee_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightKnee_slider, self.velocityFrontalRightKnee_slider, self.positionValueFrontalRightKnee_label, self.velocityValueFrontalRightKnee_label, 'FrontalRightKnee', 'rightLeg'))
        self.velocityFrontalRightKnee_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightKnee_slider, self.velocityFrontalRightKnee_slider, self.positionValueFrontalRightKnee_label, self.velocityValueFrontalRightKnee_label, 'FrontalRightKnee', 'rightLeg'))
        self.positionFrontalRightAnkle_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightAnkle_slider, self.velocityFrontalRightAnkle_slider, self.positionValueFrontalRightAnkle_label, self.velocityValueFrontalRightAnkle_label, 'FrontalRightAnkle', 'rightLeg'))
        self.velocityFrontalRightAnkle_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalRightAnkle_slider, self.velocityFrontalRightAnkle_slider, self.positionValueFrontalRightAnkle_label, self.velocityValueFrontalRightAnkle_label, 'FrontalRightAnkle', 'rightLeg'))
        self.positionSagittalRightAnkle_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalRightAnkle_slider, self.velocitySagittalRightAnkle_slider, self.positionValueSagittalRightAnkle_label, self.velocityValueSagittalRightAnkle_label, 'SagittalRightAnkle', 'rightLeg'))
        self.velocitySagittalRightAnkle_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalRightAnkle_slider, self.velocitySagittalRightAnkle_slider, self.positionValueSagittalRightAnkle_label, self.velocityValueSagittalRightAnkle_label, 'SagittalRightAnkle', 'rightLeg'))

        self.positionAxialLeftHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialLeftHip_slider, self.velocityAxialLeftHip_slider, self.positionValueAxialLeftHip_label, self.velocityValueAxialLeftHip_label, 'AxialLeftHip', 'leftLeg'))
        self.velocityAxialLeftHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionAxialLeftHip_slider, self.velocityAxialLeftHip_slider, self.positionValueAxialLeftHip_label, self.velocityValueAxialLeftHip_label, 'AxialLeftHip', 'leftLeg'))
        self.positionSagittalLeftHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalLeftHip_slider, self.velocitySagittalLeftHip_slider, self.positionValueSagittalLeftHip_label, self.velocityValueSagittalLeftHip_label, 'SagittalLeftHip', 'leftLeg'))
        self.velocitySagittalLeftHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalLeftHip_slider, self.velocitySagittalLeftHip_slider, self.positionValueSagittalLeftHip_label, self.velocityValueSagittalLeftHip_label, 'SagittalLeftHip', 'leftLeg'))
        self.positionFrontalLeftHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftHip_slider, self.velocityFrontalLeftHip_slider, self.positionValueFrontalLeftHip_label, self.velocityValueFrontalLeftHip_label, 'FrontalLeftHip', 'leftLeg'))
        self.velocityFrontalLeftHip_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftHip_slider, self.velocityFrontalLeftHip_slider, self.positionValueFrontalLeftHip_label, self.velocityValueFrontalLeftHip_label, 'FrontalLeftHip', 'leftLeg'))
        self.positionFrontalLeftKnee_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftKnee_slider, self.velocityFrontalLeftKnee_slider, self.positionValueFrontalLeftKnee_label, self.velocityValueFrontalLeftKnee_label, 'FrontalLeftKnee', 'leftLeg'))
        self.velocityFrontalLeftKnee_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftKnee_slider, self.velocityFrontalLeftKnee_slider, self.positionValueFrontalLeftKnee_label, self.velocityValueFrontalLeftKnee_label, 'FrontalLeftKnee', 'leftLeg'))
        self.positionFrontalLeftAnkle_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftAnkle_slider, self.velocityFrontalLeftAnkle_slider, self.positionValueFrontalLeftAnkle_label, self.velocityValueFrontalLeftAnkle_label, 'FrontalLeftAnkle', 'leftLeg'))
        self.velocityFrontalLeftAnkle_slider.valueChanged.connect(lambda:self.dragSlider(self.positionFrontalLeftAnkle_slider, self.velocityFrontalLeftAnkle_slider, self.positionValueFrontalLeftAnkle_label, self.velocityValueFrontalLeftAnkle_label, 'FrontalLeftAnkle', 'leftLeg'))
        self.positionSagittalLeftAnkle_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalLeftAnkle_slider, self.velocitySagittalLeftAnkle_slider, self.positionValueSagittalLeftAnkle_label, self.velocityValueSagittalLeftAnkle_label, 'SagittalLeftAnkle', 'leftLeg'))
        self.velocitySagittalLeftAnkle_slider.valueChanged.connect(lambda:self.dragSlider(self.positionSagittalLeftAnkle_slider, self.velocitySagittalLeftAnkle_slider, self.positionValueSagittalLeftAnkle_label, self.velocityValueSagittalLeftAnkle_label, 'SagittalLeftAnkle', 'leftLeg'))

        self.show()

    def dragSlider(self, positionSlider, velocitySlider, positionValueLabel, velocityValueLabel, articulation, extremity):
        
        positionValueLabel.setText(str(positionSlider.value() - 90) + " degrees")
        velocityValueLabel.setText(str(velocitySlider.value()/100) + " radians")

        self.subscriber.publishPosition(articulation, str(positionSlider.value() - 90), str(velocitySlider.value()), extremity)

    
def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()