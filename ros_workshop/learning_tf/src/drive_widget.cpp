#include <stdio.h>
#include <math.h>

//#include <QPainter>
#include <QMouseEvent>

#include "drive_widget.h"

namespace learning_tf {

// BEGIN_TUTORIAL
// The DriveWidget constructor does the normal Qt thing of
// passing the parent widget to the superclass constructor, then
// initializing the member variables.
DriveWidget::DriveWidget(QWidget* parent) :
		QWidget(parent), linear_velocity_(0), angular_velocity_(0), linear_scale_(5), angular_scale_(2) {
	// create push button objects
	pushButton_1 = new QPushButton("Forward", this);
//	pushButton_1->setShortcut();
	pushButton_2 = new QPushButton("Backward", this);
	pushButton_3 = new QPushButton("LeftTurn", this);
	pushButton_4 = new QPushButton("RightTurn", this);
//	pushButton_4->setCheckable(true);
//	pushButton_5 = new QPushButton("Stop",this);
	connect(pushButton_1, SIGNAL( pressed()), this, SLOT( buttonPressed()));
	connect(pushButton_2, SIGNAL( pressed()), this, SLOT( buttonPressed()));
	connect(pushButton_3, SIGNAL( pressed()), this, SLOT( buttonPressed()));
	connect(pushButton_4, SIGNAL( pressed()), this, SLOT( buttonPressed()));
	connect(pushButton_1, SIGNAL( released()), this, SLOT( buttonReleased()));
	connect(pushButton_2, SIGNAL( released()), this, SLOT( buttonReleased()));
	connect(pushButton_3, SIGNAL( released()), this, SLOT( buttonReleased()));
	connect(pushButton_4, SIGNAL( released()), this, SLOT( buttonReleased()));
}

// This paintEvent() is complex because of the drawing of the two
// arc-arrows representing wheel motion.  It is not particularly
// relevant to learning how to make an RViz plugin, so I will kind of
// skim it.
void DriveWidget::paintEvent(QPaintEvent* event) {
	int w = width();
	int h = height();
	int size = (w > h) ? h : w;
	int button_size = size / 4;
	int centre_offset = size / 4;

	int h1 = w / 2 - button_size / 2;
	int v1 = h / 2 - centre_offset - button_size / 2;
	pushButton_1->setGeometry(QRect(h1, v1, button_size, button_size));
	QFont font;
	font.setPointSize(size / 6);
	pushButton_1->setFont(font);
	pushButton_1->setText("F");

	int h2 = w / 2 - button_size / 2;
	int v2 = h / 2 + centre_offset - button_size / 2;
	pushButton_2->setGeometry(QRect(h2, v2, button_size, button_size));
	pushButton_2->setFont(font);
	pushButton_2->setText("B");

	int h3 = w / 2 - centre_offset - button_size / 2;
	int v3 = h / 2 - button_size / 2;
	pushButton_3->setGeometry(QRect(h3, v3, button_size, button_size));
	pushButton_3->setFont(font);
	pushButton_3->setText("L");

	int h4 = w / 2 + centre_offset - button_size / 2;
	int v4 = h / 2 - button_size / 2;
	pushButton_4->setGeometry(QRect(h4, v4, button_size, button_size));
	pushButton_4->setFont(font);
	pushButton_4->setText("R");

//	int h5 = w / 2 - button_size / 2;
//	int v5 = h / 2 - button_size / 2;
//	pushButton_5->setGeometry(QRect(h5, v5, button_size, button_size));
//	pushButton_5->setFont(font);
//	pushButton_5->setText("S");
//	update();
}

void DriveWidget::buttonPressed() {
	if (pushButton_1->isDown())
		linear_velocity_ = linear_scale_;
	else if (pushButton_2->isDown())
		linear_velocity_ = -linear_scale_;
	if (pushButton_3->isDown())
		angular_velocity_ = angular_scale_;
	else if (pushButton_4->isDown())
		angular_velocity_ = -angular_scale_;
	Q_EMIT outputVelocity(linear_velocity_, angular_velocity_);
	update();
}

void DriveWidget::buttonReleased() {
	stop();
}

// How to stop: emit velocities of 0!
void DriveWidget::stop() {
	linear_velocity_ = 0;
	angular_velocity_ = 0;
	Q_EMIT outputVelocity(linear_velocity_, angular_velocity_);
	update();
}

} // end namespace leaning_tf
