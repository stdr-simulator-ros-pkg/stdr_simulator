/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
   
   Authors : 
   * Manos Tsardoulias, etsardou@gmail.com
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com 
******************************************************************************/

#ifndef STDR_TOOLS
#define STDR_TOOLS

#include <ros/package.h>
#include "ros/ros.h"
#include <tf/transform_listener.h>

#include <QtUiTools/QUiLoader>

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QTime>

#include <QtGui/QMenu>
#include <QtGui/QApplication>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QImage>
#include <QtGui/QFileDialog>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
#include <QtGui/QPainter>
#include <QtGui/QPixmap>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QScrollBar>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
#include <QtGui/QTreeWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include <QtGui/QWindowsStyle>
#include <QtGui/QInputDialog>
#include <QtGui/QMouseEvent>
#include <QtGui/QMessageBox>
#include <QtGui/QTimeEdit>

#include <stdr_msgs/RobotIndexedVectorMsg.h>
#include <stdr_msgs/RobotIndexedMsg.h>
#include <stdr_msgs/RobotMsg.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>

#define STDR_PI 3.14159

namespace stdr_gui_tools
{
	std::string getRosPackagePath(std::string package);
	QString getLiteralTime(int ms);
	void printSonarMsg(stdr_msgs::SonarSensorMsg &msg);
	void printPose2D(geometry_msgs::Pose2D &msg);
}

#endif
