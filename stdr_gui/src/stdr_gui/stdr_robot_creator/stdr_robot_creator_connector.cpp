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

#include "stdr_gui/stdr_robot_creator/stdr_robot_creator_connector.h"

namespace stdr_gui{
	unsigned int RobotCreatorConnector::laserNumber=0;
	unsigned int RobotCreatorConnector::sonarNumber=0;
	unsigned int RobotCreatorConnector::rfidNumber=0;
	
	RobotCreatorConnector::RobotCreatorConnector(int argc, char **argv):
		QObject(),
		loader(argc,argv)
	{
		this->argc=argc;
		this->argv=argv;
		
		QObject::connect(loader.robotTreeWidget,SIGNAL(itemClicked(QTreeWidgetItem *, int)),this,SLOT(treeItemClicked(QTreeWidgetItem *, int)));
		QObject::connect(loader.laserPropLoader.laserUpdateButton,SIGNAL(clicked(bool)),this,SLOT(updateLaser()));
		QObject::connect(loader.sonarPropLoader.pushButton,SIGNAL(clicked(bool)),this,SLOT(updateSonar()));
		QObject::connect(loader.rfidAntennaPropLoader.pushButton,SIGNAL(clicked(bool)),this,SLOT(updateRfid()));
		QObject::connect(loader.saveRobotButton,SIGNAL(clicked(bool)),this,SLOT(saveRobot()));
		QObject::connect(loader.loadRobotButton,SIGNAL(clicked(bool)),this,SLOT(loadRobot()));
		QObject::connect(loader.cancelButton,SIGNAL(clicked(bool)),this,SLOT(closeRobotCreator()));

		climax=-1;
	}
	
	void RobotCreatorConnector::initialise(void){
		newRobotMsg=stdr_msgs::RobotMsg();
		
		loader.robotInfoShape.setText(0,"Shape");
		loader.robotInfoShape.setText(1,"Circle");
		loader.robotInfoWidth.setText(0,"Width");
		loader.robotInfoWidth.setText(1,"0.30");
		loader.robotInfoLength.setText(0,"Length");
		loader.robotInfoLength.setText(1,"0.30");
		
		newRobotMsg.radius=0.30;
		
		unsigned int laserCount=loader.lasersNode.childCount();
		unsigned int sonarCount=loader.sonarsNode.childCount();
		unsigned int rfidCount=loader.rfidAntennasNode.childCount();
		
		for(int i=laserCount-1;i>=0;i--)
			deleteTreeNode(loader.lasersNode.child(i));
		for(int i=sonarCount-1;i>=0;i--)
			deleteTreeNode(loader.sonarsNode.child(i));
		for(int i=rfidCount-1;i>=0;i--)
			deleteTreeNode(loader.rfidAntennasNode.child(i));
		
		RobotCreatorConnector::laserNumber=0;
		RobotCreatorConnector::sonarNumber=0;
		RobotCreatorConnector::rfidNumber=0;
		
		updateRobotPreview();
		
		loader.show();
	}

	void RobotCreatorConnector::treeItemClicked ( QTreeWidgetItem * item, int column ){
		if(item==&loader.robotNode && column==2)		// Robot edit clicked
			loader.robotPropLoader.show();
		if(item==&loader.kinematicNode && column==2)	//	Kinematic edit clicked
			loader.kinematicPropLoader.show();
		if(item==&loader.lasersNode && column==2)	//	Add laser clicked
			addLaser();
		if(item->parent()==&loader.lasersNode && column==2)	//	Erase a laser
			eraseLaser(item);
		if(item->parent()==&loader.lasersNode && column==1)	//	Edit a laser
			editLaser(item);
		if(item==&loader.sonarsNode && column==2)	//	Add a sonar
			addSonar();
		if(item==&loader.rfidAntennasNode && column==2)	//	Add a rfid antenna
			addRfidAntenna();
		if(item->parent()==&loader.sonarsNode && column==2)	//	Erase a sonar
			eraseSonar(item);
		if(item->parent()==&loader.rfidAntennasNode && column==2)	//	Erase a rfid antenna
			eraseRfid(item);
		if(item->parent()==&loader.sonarsNode && column==1)	//	Edit a sonar
			editSonar(item);
		if(item->parent()==&loader.rfidAntennasNode && column==1)	//	Edit a rfid antenna
			editRfid(item);
	}
	
	void RobotCreatorConnector::addLaser(void){
		QString laserFrameId=QString("laser_")+QString().setNum(RobotCreatorConnector::laserNumber++);
		
		stdr_msgs::LaserSensorMsg lmsg;
		lmsg.frame_id=laserFrameId.toStdString();
		lmsg.maxAngle=135;
		lmsg.minAngle=-135;
		lmsg.maxRange=4.0;
		lmsg.minRange=0.0;
		lmsg.pose.x=0;
		lmsg.pose.y=0;
		lmsg.pose.theta=0;
		lmsg.noise.noiseMean=0;
		lmsg.noise.noiseStd=0;
		
		newRobotMsg.laserSensors.push_back(lmsg);
		
		QTreeWidgetItem	*lnode;
		lnode=new QTreeWidgetItem();
		lnode->setText(0,laserFrameId);
		lnode->setIcon(1,loader.editIcon);
		lnode->setIcon(2,loader.removeIcon);

		QTreeWidgetItem *angleSpan,*orientation,*maxRange,*minRange,*noiseMean,*noiseStd,*poseX,*poseY;
		angleSpan=new QTreeWidgetItem();
		orientation=new QTreeWidgetItem();
		maxRange=new QTreeWidgetItem();
		minRange=new QTreeWidgetItem();
		noiseMean=new QTreeWidgetItem();
		noiseStd=new QTreeWidgetItem();
		poseX=new QTreeWidgetItem();
		poseY=new QTreeWidgetItem();
		angleSpan->setText(0,QString("Angle span"));
		orientation->setText(0,QString("Orientation"));
		maxRange->setText(0,QString("Max range"));
		minRange->setText(0,QString("Min range"));
		noiseMean->setText(0,QString("Noise mean"));
		noiseStd->setText(0,QString("Noise std"));
		poseX->setText(0,QString("Pose - x"));
		poseY->setText(0,QString("Pose - y"));
		angleSpan->setText(1,QString().setNum(lmsg.maxAngle-lmsg.minAngle));
		orientation->setText(1,QString().setNum(lmsg.maxAngle+lmsg.minAngle));
		maxRange->setText(1,QString().setNum(lmsg.maxRange));
		minRange->setText(1,QString().setNum(lmsg.minRange));
		noiseMean->setText(1,QString().setNum(lmsg.noise.noiseMean));
		noiseStd->setText(1,QString().setNum(lmsg.noise.noiseStd));
		poseX->setText(1,QString().setNum(lmsg.pose.x));
		poseY->setText(1,QString().setNum(lmsg.pose.y));
		
		lnode->addChild(angleSpan);
		lnode->addChild(orientation);
		lnode->addChild(maxRange);
		lnode->addChild(minRange);
		lnode->addChild(noiseMean);
		lnode->addChild(noiseStd);
		lnode->addChild(poseX);
		lnode->addChild(poseY);
		
		loader.lasersNode.addChild(lnode);
		
		lnode->setExpanded(false);
		
		updateRobotPreview();
	}
	
	void RobotCreatorConnector::addSonar(void){
		QString sonarFrameId=QString("sonar_")+QString().setNum(RobotCreatorConnector::sonarNumber++);
		
		stdr_msgs::SonarSensorMsg smsg;
		smsg.frame_id=sonarFrameId.toStdString();
		smsg.maxRange=3.0;
		smsg.minRange=0.3;
		smsg.coneAngle=50.0;
		smsg.pose.x=0;
		smsg.pose.y=0;
		smsg.pose.theta=0;
		smsg.noise.noiseMean=0;
		smsg.noise.noiseStd=0;
		
		newRobotMsg.sonarSensors.push_back(smsg);
		
		QTreeWidgetItem	*snode;
		snode=new QTreeWidgetItem();
		snode->setText(0,sonarFrameId);
		snode->setIcon(1,loader.editIcon);
		snode->setIcon(2,loader.removeIcon);

		QTreeWidgetItem *coneAngle,*orientation,*maxRange,*minRange,*noiseMean,*noiseStd,*poseX,*poseY;
		coneAngle=new QTreeWidgetItem();
		orientation=new QTreeWidgetItem();
		maxRange=new QTreeWidgetItem();
		minRange=new QTreeWidgetItem();
		noiseMean=new QTreeWidgetItem();
		noiseStd=new QTreeWidgetItem();
		poseX=new QTreeWidgetItem();
		poseY=new QTreeWidgetItem();
		coneAngle->setText(0,QString("Cone span"));
		orientation->setText(0,QString("Orientation"));
		maxRange->setText(0,QString("Max range"));
		minRange->setText(0,QString("Min range"));
		noiseMean->setText(0,QString("Noise mean"));
		noiseStd->setText(0,QString("Noise std"));
		poseX->setText(0,QString("Pose - x"));
		poseY->setText(0,QString("Pose - y"));
		coneAngle->setText(1,QString().setNum(smsg.coneAngle));
		orientation->setText(1,QString().setNum(smsg.pose.theta));
		maxRange->setText(1,QString().setNum(smsg.maxRange));
		minRange->setText(1,QString().setNum(smsg.minRange));
		noiseMean->setText(1,QString().setNum(smsg.noise.noiseMean));
		noiseStd->setText(1,QString().setNum(smsg.noise.noiseStd));
		poseX->setText(1,QString().setNum(smsg.pose.x));
		poseY->setText(1,QString().setNum(smsg.pose.y));
		
		snode->addChild(coneAngle);
		snode->addChild(orientation);
		snode->addChild(maxRange);
		snode->addChild(minRange);
		snode->addChild(noiseMean);
		snode->addChild(noiseStd);
		snode->addChild(poseX);
		snode->addChild(poseY);
		
		loader.sonarsNode.addChild(snode);
		
		snode->setExpanded(false);
		
		updateRobotPreview();
	}
	
	void RobotCreatorConnector::addRfidAntenna(void){
		QString rfidFrameId=QString("rfid_antenna_")+QString().setNum(RobotCreatorConnector::rfidNumber++);
		
		stdr_msgs::RfidSensorMsg smsg;
		smsg.frame_id=rfidFrameId.toStdString();
		smsg.maxRange=3.0;
		smsg.angleSpan=360.0;
		smsg.pose.x=0;
		smsg.pose.y=0;
		smsg.pose.theta=0;
		smsg.signalCutoff=0;
		
		newRobotMsg.rfidSensors.push_back(smsg);
		
		QTreeWidgetItem	*snode;
		snode=new QTreeWidgetItem();
		snode->setText(0,rfidFrameId);
		snode->setIcon(1,loader.editIcon);
		snode->setIcon(2,loader.removeIcon);

		QTreeWidgetItem *angleSpan,*orientation,*maxRange,*poseX,*poseY,*signalCutoff;
		angleSpan=new QTreeWidgetItem();
		orientation=new QTreeWidgetItem();
		maxRange=new QTreeWidgetItem();
		poseX=new QTreeWidgetItem();
		poseY=new QTreeWidgetItem();
		signalCutoff=new QTreeWidgetItem();
		
		angleSpan->setText(0,QString("Angle span"));
		orientation->setText(0,QString("Orientation"));
		maxRange->setText(0,QString("Max range"));
		poseX->setText(0,QString("Pose - x"));
		poseY->setText(0,QString("Pose - y"));
		signalCutoff->setText(0,QString("Signal cutoff"));
		
		angleSpan->setText(1,QString().setNum(smsg.angleSpan));
		orientation->setText(1,QString().setNum(smsg.pose.theta));
		maxRange->setText(1,QString().setNum(smsg.maxRange));
		poseX->setText(1,QString().setNum(smsg.pose.x));
		poseY->setText(1,QString().setNum(smsg.pose.y));
		signalCutoff->setText(1,QString().setNum(smsg.signalCutoff));
		
		snode->addChild(angleSpan);
		snode->addChild(orientation);
		snode->addChild(maxRange);
		snode->addChild(poseX);
		snode->addChild(poseY);
		snode->addChild(signalCutoff);
		
		loader.rfidAntennasNode.addChild(snode);
		
		snode->setExpanded(false);
		
		updateRobotPreview();
	}
	
	void RobotCreatorConnector::eraseLaser(QTreeWidgetItem *item){
		unsigned int laserFrameId=searchLaser(item->text(0));
		if(laserFrameId==-1) 
			return;
		newRobotMsg.laserSensors.erase(newRobotMsg.laserSensors.begin()+laserFrameId);
		deleteTreeNode(item);
		updateRobotPreview();
	}
	
	void RobotCreatorConnector::eraseSonar(QTreeWidgetItem *item){
		unsigned int sonarFrameId=searchSonar(item->text(0));
		if(sonarFrameId==-1) 
			return;
		newRobotMsg.sonarSensors.erase(newRobotMsg.sonarSensors.begin()+sonarFrameId);
		deleteTreeNode(item);
		updateRobotPreview();
	}
	
	void RobotCreatorConnector::eraseRfid(QTreeWidgetItem *item){
		unsigned int rfidFrameId=searchRfid(item->text(0));
		if(rfidFrameId==-1) 
			return;
		newRobotMsg.rfidSensors.erase(newRobotMsg.rfidSensors.begin()+rfidFrameId);
		deleteTreeNode(item);
		updateRobotPreview();
	}
	
	void RobotCreatorConnector::editLaser(QTreeWidgetItem *item){
		unsigned int laserFrameId=searchLaser(item->text(0));
		if(laserFrameId==-1) 
			return;
			
		loader.laserPropLoader.laserMaxDistance->setText(QString().setNum(newRobotMsg.laserSensors[laserFrameId].maxRange));
		loader.laserPropLoader.laserMinDistance->setText(QString().setNum(newRobotMsg.laserSensors[laserFrameId].minRange));
		loader.laserPropLoader.laserAngleSpan->setText(QString().setNum(newRobotMsg.laserSensors[laserFrameId].maxAngle-newRobotMsg.laserSensors[laserFrameId].minAngle));
		loader.laserPropLoader.laserOrientation->setText(QString().setNum(newRobotMsg.laserSensors[laserFrameId].pose.theta));
		loader.laserPropLoader.laserNoiseMean->setText(QString().setNum(newRobotMsg.laserSensors[laserFrameId].noise.noiseMean));
		loader.laserPropLoader.laserNoiseStd->setText(QString().setNum(newRobotMsg.laserSensors[laserFrameId].noise.noiseStd));
		
		loader.laserPropLoader.setWindowTitle(QApplication::translate("LaserProperties", item->text(0).toStdString().c_str(), 0, QApplication::UnicodeUTF8));
		
		currentLaser=item;
		
		loader.laserPropLoader.show();
	}
	
	void RobotCreatorConnector::editSonar(QTreeWidgetItem *item){
		unsigned int sonarFrameId=searchSonar(item->text(0));
		if(sonarFrameId==-1) 
			return;
			
		loader.sonarPropLoader.sonarMaxDistance->setText(QString().setNum(newRobotMsg.sonarSensors[sonarFrameId].maxRange));
		loader.sonarPropLoader.sonarMinDistance->setText(QString().setNum(newRobotMsg.sonarSensors[sonarFrameId].minRange));
		loader.sonarPropLoader.sonarX->setText(QString().setNum(newRobotMsg.sonarSensors[sonarFrameId].pose.x));
		loader.sonarPropLoader.sonarY->setText(QString().setNum(newRobotMsg.sonarSensors[sonarFrameId].pose.y));
		loader.sonarPropLoader.sonarConeSpan->setText(QString().setNum(newRobotMsg.sonarSensors[sonarFrameId].coneAngle));
		loader.sonarPropLoader.sonarNoiseMean->setText(QString().setNum(newRobotMsg.sonarSensors[sonarFrameId].noise.noiseMean));
		loader.sonarPropLoader.sonarNoiseStd->setText(QString().setNum(newRobotMsg.sonarSensors[sonarFrameId].noise.noiseStd));
		loader.sonarPropLoader.sonarOrientation->setText(QString().setNum(newRobotMsg.sonarSensors[sonarFrameId].pose.theta));
		
		loader.sonarPropLoader.setWindowTitle(QApplication::translate("SonarProperties", item->text(0).toStdString().c_str(), 0, QApplication::UnicodeUTF8));
		
		currentSonar=item;
		
		loader.sonarPropLoader.show();
	}
	
	void RobotCreatorConnector::editRfid(QTreeWidgetItem *item){
		unsigned int frameId=searchRfid(item->text(0));
		if(frameId==-1) 
			return;
			
		loader.rfidAntennaPropLoader.rfidMaxDistance->setText(QString().setNum(newRobotMsg.rfidSensors[frameId].maxRange));
		loader.rfidAntennaPropLoader.rfidX->setText(QString().setNum(newRobotMsg.rfidSensors[frameId].pose.x));
		loader.rfidAntennaPropLoader.rfidY->setText(QString().setNum(newRobotMsg.rfidSensors[frameId].pose.y));
		loader.rfidAntennaPropLoader.rfidAngleSpan->setText(QString().setNum(newRobotMsg.rfidSensors[frameId].angleSpan));
		loader.rfidAntennaPropLoader.rfidOrientation->setText(QString().setNum(newRobotMsg.rfidSensors[frameId].pose.theta));
		loader.rfidAntennaPropLoader.rfidSignalCutoff->setText(QString().setNum(newRobotMsg.rfidSensors[frameId].signalCutoff));
		
		loader.rfidAntennaPropLoader.setWindowTitle(QApplication::translate("RfidAntennaProperties", item->text(0).toStdString().c_str(), 0, QApplication::UnicodeUTF8));
		
		currentRfid=item;
		
		loader.rfidAntennaPropLoader.show();
	}
	
	int RobotCreatorConnector::searchLaser(QString frameId){
		for(unsigned int i=0;i<newRobotMsg.laserSensors.size();i++)
			if(newRobotMsg.laserSensors[i].frame_id==frameId.toStdString())
				return i;
		return -1;
	}
	
	int RobotCreatorConnector::searchSonar(QString frameId){
		for(unsigned int i=0;i<newRobotMsg.sonarSensors.size();i++)
			if(newRobotMsg.sonarSensors[i].frame_id==frameId.toStdString())
				return i;
		return -1;
	}
	
	int RobotCreatorConnector::searchRfid(QString frameId){
		for(unsigned int i=0;i<newRobotMsg.rfidSensors.size();i++)
			if(newRobotMsg.rfidSensors[i].frame_id==frameId.toStdString())
				return i;
		return -1;
	}
			
	void RobotCreatorConnector::updateLaser(void){
		unsigned int laserFrameId=searchLaser(currentLaser->text(0));
		if(laserFrameId==-1) 
			return;
		for(unsigned int i=0;i<currentLaser->childCount();i++){
			if(currentLaser->child(i)->text(0)==QString("Angle span")){
				currentLaser->child(i)->setText(1,loader.laserPropLoader.laserAngleSpan->text());
				float angleSpan=loader.laserPropLoader.laserAngleSpan->text().toFloat()/2.0;
				newRobotMsg.laserSensors[laserFrameId].minAngle=-angleSpan;
				newRobotMsg.laserSensors[laserFrameId].maxAngle=angleSpan;
			}
			else if(currentLaser->child(i)->text(0)==QString("Orientation")){
				float orientation=loader.laserPropLoader.laserOrientation->text().toFloat();
				currentLaser->child(i)->setText(1,QString().setNum(orientation));
				newRobotMsg.laserSensors[laserFrameId].minAngle+=orientation;
				newRobotMsg.laserSensors[laserFrameId].maxAngle+=orientation;
				newRobotMsg.laserSensors[laserFrameId].pose.theta=orientation;
			}
			else if(currentLaser->child(i)->text(0)==QString("Max range")){
				float maxRange=loader.laserPropLoader.laserMaxDistance->text().toFloat();
				currentLaser->child(i)->setText(1,QString().setNum(maxRange));
				newRobotMsg.laserSensors[laserFrameId].maxRange=maxRange;
			}
			else if(currentLaser->child(i)->text(0)==QString("Min range")){
				float minRange=loader.laserPropLoader.laserMinDistance->text().toFloat();
				currentLaser->child(i)->setText(1,QString().setNum(minRange));
				newRobotMsg.laserSensors[laserFrameId].minRange=minRange;
			}
			else if(currentLaser->child(i)->text(0)==QString("Noise mean")){
				float noiseMean=loader.laserPropLoader.laserNoiseMean->text().toFloat();
				currentLaser->child(i)->setText(1,QString().setNum(noiseMean));
				newRobotMsg.laserSensors[laserFrameId].noise.noiseMean=noiseMean;
			}
			else if(currentLaser->child(i)->text(0)==QString("Noise std")){
				float noiseStd=loader.laserPropLoader.laserNoiseStd->text().toFloat();
				currentLaser->child(i)->setText(1,QString().setNum(noiseStd));
				newRobotMsg.laserSensors[laserFrameId].noise.noiseStd=noiseStd;
			}
			else if(currentLaser->child(i)->text(0)==QString("Pose - x")){
				float dx=loader.laserPropLoader.laserTranslationX->text().toFloat();
				currentLaser->child(i)->setText(1,QString().setNum(dx));
				newRobotMsg.laserSensors[laserFrameId].pose.x=dx;
			}
			else if(currentLaser->child(i)->text(0)==QString("Pose - y")){
				float dy=loader.laserPropLoader.laserTranslationY->text().toFloat();
				currentLaser->child(i)->setText(1,QString().setNum(dy));
				newRobotMsg.laserSensors[laserFrameId].pose.y=dy;
			}
		}
		
		//printLaserMsg(newRobotMsg.laserSensors[laserFrameId]);
		
		loader.laserPropLoader.hide();
		
		updateRobotPreview();
	}

	void RobotCreatorConnector::updateSonar(void){
		unsigned int frameId=searchSonar(currentSonar->text(0));
		if(frameId==-1) 
			return;
		for(unsigned int i=0;i<currentSonar->childCount();i++){
			if(currentSonar->child(i)->text(0)==QString("Cone span")){
				currentSonar->child(i)->setText(1,loader.sonarPropLoader.sonarConeSpan->text());
				newRobotMsg.sonarSensors[frameId].coneAngle=loader.sonarPropLoader.sonarConeSpan->text().toFloat();
			}
			else if(currentSonar->child(i)->text(0)==QString("Orientation")){
				float orientation=loader.sonarPropLoader.sonarOrientation->text().toFloat();
				currentSonar->child(i)->setText(1,QString().setNum(orientation));
				newRobotMsg.sonarSensors[frameId].pose.theta=orientation;
			}
			else if(currentSonar->child(i)->text(0)==QString("Max range")){
				float maxRange=loader.sonarPropLoader.sonarMaxDistance->text().toFloat();
				currentSonar->child(i)->setText(1,QString().setNum(maxRange));
				newRobotMsg.sonarSensors[frameId].maxRange=maxRange;
			}
			else if(currentSonar->child(i)->text(0)==QString("Min range")){
				float minRange=loader.sonarPropLoader.sonarMinDistance->text().toFloat();
				currentSonar->child(i)->setText(1,QString().setNum(minRange));
				newRobotMsg.sonarSensors[frameId].minRange=minRange;
			}
			else if(currentSonar->child(i)->text(0)==QString("Noise mean")){
				float noiseMean=loader.sonarPropLoader.sonarNoiseMean->text().toFloat();
				currentSonar->child(i)->setText(1,QString().setNum(noiseMean));
				newRobotMsg.sonarSensors[frameId].noise.noiseMean=noiseMean;
			}
			else if(currentSonar->child(i)->text(0)==QString("Noise std")){
				float noiseStd=loader.sonarPropLoader.sonarNoiseStd->text().toFloat();
				currentSonar->child(i)->setText(1,QString().setNum(noiseStd));
				newRobotMsg.sonarSensors[frameId].noise.noiseStd=noiseStd;
			}
			else if(currentSonar->child(i)->text(0)==QString("Pose - x")){
				float dx=loader.sonarPropLoader.sonarX->text().toFloat();
				currentSonar->child(i)->setText(1,QString().setNum(dx));
				newRobotMsg.sonarSensors[frameId].pose.x=dx;
			}
			else if(currentSonar->child(i)->text(0)==QString("Pose - y")){
				float dy=loader.sonarPropLoader.sonarY->text().toFloat();
				currentSonar->child(i)->setText(1,QString().setNum(dy));
				newRobotMsg.sonarSensors[frameId].pose.y=dy;
			}
		}

		loader.sonarPropLoader.hide();
		
		updateRobotPreview();
	}
	
	void RobotCreatorConnector::updateRfid(void){
		unsigned int frameId=searchRfid(currentRfid->text(0));
		if(frameId==-1) 
			return;
		for(unsigned int i=0;i<currentRfid->childCount();i++){
			if(currentRfid->child(i)->text(0)==QString("Angle span")){
				currentRfid->child(i)->setText(1,loader.rfidAntennaPropLoader.rfidAngleSpan->text());
				newRobotMsg.rfidSensors[frameId].angleSpan=loader.rfidAntennaPropLoader.rfidAngleSpan->text().toFloat();
			}
			else if(currentRfid->child(i)->text(0)==QString("Orientation")){
				float orientation=loader.rfidAntennaPropLoader.rfidOrientation->text().toFloat();
				currentRfid->child(i)->setText(1,QString().setNum(orientation));
				newRobotMsg.rfidSensors[frameId].pose.theta=orientation;
			}
			else if(currentRfid->child(i)->text(0)==QString("Max range")){
				float maxRange=loader.rfidAntennaPropLoader.rfidMaxDistance->text().toFloat();
				currentRfid->child(i)->setText(1,QString().setNum(maxRange));
				newRobotMsg.rfidSensors[frameId].maxRange=maxRange;
			}
			else if(currentRfid->child(i)->text(0)==QString("Pose - x")){
				float dx=loader.rfidAntennaPropLoader.rfidX->text().toFloat();
				currentRfid->child(i)->setText(1,QString().setNum(dx));
				newRobotMsg.rfidSensors[frameId].pose.x=dx;
			}
			else if(currentRfid->child(i)->text(0)==QString("Pose - y")){
				float dy=loader.rfidAntennaPropLoader.rfidY->text().toFloat();
				currentRfid->child(i)->setText(1,QString().setNum(dy));
				newRobotMsg.rfidSensors[frameId].pose.y=dy;
			}
			else if(currentRfid->child(i)->text(0)==QString("Signal cutoff")){
				float signal=loader.rfidAntennaPropLoader.rfidSignalCutoff->text().toFloat();
				currentRfid->child(i)->setText(1,QString().setNum(signal));
				newRobotMsg.rfidSensors[frameId].signalCutoff=signal;
			}
		}

		loader.rfidAntennaPropLoader.hide();
		
		updateRobotPreview();
	}

	void RobotCreatorConnector::printLaserMsg(stdr_msgs::LaserSensorMsg msg){
		ROS_ERROR("Laser : %s",msg.frame_id.c_str());
		ROS_ERROR("\tMax angle : %f",msg.maxAngle);
		ROS_ERROR("\tMin angle : %f",msg.minAngle);
		ROS_ERROR("\tMax range : %f",msg.maxRange);
		ROS_ERROR("\tMin range : %f",msg.minRange);
		ROS_ERROR("\tRelative pose : %f %f %f",msg.pose.x,msg.pose.y,msg.pose.theta);
		ROS_ERROR("\tNoise : %f %f",msg.noise.noiseMean,msg.noise.noiseStd);
	}
	
	void RobotCreatorConnector::deleteTreeNode(QTreeWidgetItem *item){
		int count=item->childCount();
		for(int i=count-1;i>=0;i--)
			deleteTreeNode(item->child(i));
		delete item;
	}
		
	void RobotCreatorConnector::updateRobotPreview(void){
		
		loader.robotPreviewImage.fill(QColor(220,220,220,1));
		
		climax=-1;
		if(climax<newRobotMsg.radius) 
			climax=newRobotMsg.radius;
		for(unsigned int i=0;i<newRobotMsg.laserSensors.size();i++)
			if(climax<newRobotMsg.laserSensors[i].maxRange) 
				climax=newRobotMsg.laserSensors[i].maxRange;
		for(unsigned int i=0;i<newRobotMsg.sonarSensors.size();i++)
			if(climax<newRobotMsg.sonarSensors[i].maxRange) 
				climax=newRobotMsg.sonarSensors[i].maxRange;
		for(unsigned int i=0;i<newRobotMsg.rfidSensors.size();i++)
			if(climax<newRobotMsg.rfidSensors[i].maxRange) 
				climax=newRobotMsg.rfidSensors[i].maxRange;
		
		climax=230.0/climax;
		drawRobot(newRobotMsg.radius);
		drawLasers();
		drawSonars();
		drawRfidAntennas();
	}
	
	void RobotCreatorConnector::drawRobot(float radius){
		QPainter painter(&loader.robotPreviewImage);
		painter.setPen(Qt::blue);
		painter.drawEllipse(250-radius*climax,250-radius*climax,radius*climax*2,radius*climax*2);
		painter.drawLine(250,250,250+radius*climax*1.05,250);
		loader.robotPreviewLabel->setPixmap(QPixmap().fromImage(loader.robotPreviewImage));
	}
	
	void RobotCreatorConnector::drawRobot(float length,float width){
		
	}
	
	void RobotCreatorConnector::drawRobot(std::vector<std::pair<float,float> > geometry){
		
	}

	void RobotCreatorConnector::drawLasers(void){
		QPainter painter(&loader.robotPreviewImage);
		QBrush brush(QColor(0,200,0,50));
		painter.setBrush(brush);
		for(unsigned int i=0;i<newRobotMsg.laserSensors.size();i++){
			painter.drawPie(	250-newRobotMsg.laserSensors[i].maxRange*climax,
								250-newRobotMsg.laserSensors[i].maxRange*climax,
								newRobotMsg.laserSensors[i].maxRange*climax*2,
								newRobotMsg.laserSensors[i].maxRange*climax*2,
								newRobotMsg.laserSensors[i].minAngle*16,
								newRobotMsg.laserSensors[i].maxAngle*16-newRobotMsg.laserSensors[i].minAngle*16);
		}
		loader.robotPreviewLabel->setPixmap(QPixmap().fromImage(loader.robotPreviewImage));
	}
	
	void RobotCreatorConnector::drawSonars(void){
		QPainter painter(&loader.robotPreviewImage);
		QBrush brush(QColor(200,0,0,50));
		painter.setBrush(brush);
		for(unsigned int i=0;i<newRobotMsg.sonarSensors.size();i++){
			painter.drawPie(	250-newRobotMsg.sonarSensors[i].maxRange*climax,
								250-newRobotMsg.sonarSensors[i].maxRange*climax,
								newRobotMsg.sonarSensors[i].maxRange*climax*2,
								newRobotMsg.sonarSensors[i].maxRange*climax*2,
								(newRobotMsg.sonarSensors[i].pose.theta-newRobotMsg.sonarSensors[i].coneAngle/2.0)*16,
								(newRobotMsg.sonarSensors[i].coneAngle)*16);
		}
		loader.robotPreviewLabel->setPixmap(QPixmap().fromImage(loader.robotPreviewImage));
	}
	
	void RobotCreatorConnector::drawRfidAntennas(void){
		QPainter painter(&loader.robotPreviewImage);
		QBrush brush(QColor(0,0,200,20));
		painter.setBrush(brush);
		for(unsigned int i=0;i<newRobotMsg.rfidSensors.size();i++){
			painter.drawPie(	250-newRobotMsg.rfidSensors[i].maxRange*climax,
								250-newRobotMsg.rfidSensors[i].maxRange*climax,
								newRobotMsg.rfidSensors[i].maxRange*climax*2,
								newRobotMsg.rfidSensors[i].maxRange*climax*2,
								(newRobotMsg.rfidSensors[i].pose.theta-newRobotMsg.rfidSensors[i].angleSpan/2.0)*16,
								(newRobotMsg.rfidSensors[i].angleSpan)*16);
		}
		loader.robotPreviewLabel->setPixmap(QPixmap().fromImage(loader.robotPreviewImage));
	}

	void RobotCreatorConnector::saveRobot(void){
		emit saveRobotPressed(newRobotMsg);
		loader.hide();
	}
	
	void RobotCreatorConnector::closeRobotCreator(void){
		loader.hide();
	}
	
	void RobotCreatorConnector::loadRobot(void){
		emit loadRobotPressed(newRobotMsg);
		loader.hide();
	}
}
