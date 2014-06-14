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

#include "stdr_gui/stdr_map_loader.h"

namespace stdr_gui
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CMapLoader::CMapLoader(int argc, char **argv):
    argc_(argc),
    argv_(argv)
  {
    setupUi(this);
    internal_img_ = new QImage(100,100,QImage::Format_RGB32);
    map_min_ = QPoint(0,0);
    map_max_ = QPoint(0,0);
    zoom_ = 0;
  }
  
  /**
  @brief Captures the resize event
  @param e [QResizeEvent*] The resize event
  @return void
  **/
  void CMapLoader::resizeEvent(QResizeEvent *e)
  {
    updateImage(internal_img_);
  }
  
  /**
  @brief Return the dimensions according to the container size
  @param w [int] Image width
  @param h [int] Image height
  @return std::pair<int,int> : The size the map must be resized to
  **/
  std::pair<int,int> CMapLoader::checkDimensions(int w,int h)
  {
    float containerWidth = this->width();
    float containerHeight = this->height();
    float aspectRatio = (float)w / (float)h;
    float finalW,finalH;
    if(containerHeight * aspectRatio > containerWidth)
    {
      finalW = containerWidth;
      finalH = containerWidth / aspectRatio;
    }
    else
    {
      finalW = containerHeight * aspectRatio;
      finalH = containerHeight;
    }
    return std::pair<int,int>(finalW,finalH);
  }
  
  /**
  @brief Updates the image
  @param img [QImage*] The image to be updated
  @return void
  **/
  void CMapLoader::updateImage(QImage *img)
  {
    internal_img_ = img;
    std::pair<int,int> newDims = checkDimensions(img->width(),img->height());
    
    map->setPixmap(
      QPixmap().fromImage(
        (*img).
          copy(map_min_.x(),
              map_min_.y(),
              map_max_.x() - map_min_.x(),
              map_max_.y() - map_min_.y()).
          scaled(newDims.first,newDims.second,
              Qt::IgnoreAspectRatio,
              Qt::SmoothTransformation)));
          
    map->resize(newDims.first,newDims.second);
  }
  
  /**
  @brief Draws a grid in an image
  @param img [QImage*] The image for the grid to be drawn on
  @param resolution [float] The map resolution
  @return void
  **/
  void CMapLoader::drawGrid(QImage *img,float resolution)
  {
    QPainter painter(img);
    painter.setPen(QColor(100,100,100,150));
    int pix = 1.0 / resolution;
    for(unsigned int i = 1 ; i <= img->width() / pix + 1 ; i++)
    {
      painter.drawLine(0, i * pix, img->width() - 1, i * pix);
    }
    for(unsigned int i = 1 ; i <= img->height() / pix + 1 ; i++)
    {
      painter.drawLine(i * pix, 0, i * pix, img->height() - 1);
    }
  }
  
  /**
  @brief Updates the zoom of the image
  @param p [QPoint] The point of the zoom event
  @param zoomIn [bool] True if zoom in, false if zoom out
  @return void
  **/
  void CMapLoader::updateZoom(QPoint p,bool zoomIn)
  {
    QPoint np = getGlobalPoint(p);
    np.setY(internal_img_->height() - np.y());
    int prevZoom = zoom_;
    if(zoomIn)
    {
      zoom_++;
    }
    else 
    {
      zoom_--;
    }
    if(zoom_ < 0)
    {
      zoom_ = 0;
      return;
    }
    float intW = internal_img_->width();
    float intH = internal_img_->height();
    float newWidth = internal_img_->width() / pow(ZOOM_RATIO,zoom_);
    float newHeight = internal_img_->height() / pow(ZOOM_RATIO,zoom_);
    QPoint evOriginal = np;
    
    float xmin,xmax,ymin,ymax;
    float prevxmin, prevymin, prevWidth, prevHeight;
    prevxmin = map_min_.x();
    prevymin = map_min_.y();
    prevWidth = map_max_.x() - map_min_.x();
    prevHeight = map_max_.y() - map_min_.y();
    float xhit = evOriginal.x();
    float yhit = evOriginal.y();

    xmin = xhit - (xhit - prevxmin) * newWidth / prevWidth;
    xmax = xmin + newWidth ;
    ymin = yhit - (yhit - prevymin) * newHeight / prevHeight;
    ymax = ymin + newHeight;
    
    if(xmin < 0)
    {
      xmax += - xmin;
      xmin = 0;
    }
    else if(xmax > internal_img_->width() - 1)
    {
      xmin -= xmax - internal_img_->width() + 1;
      xmax = internal_img_->width() - 1;
    }
    if(ymin < 0)
    {
      ymax += - ymin;
      ymin = 0;
    }
    else if(ymax > internal_img_->height() - 1)
    {
      ymin -= ymax - internal_img_->height() + 1;
      ymax = internal_img_->height() - 1;
    }
    map_min_ = QPoint(xmin,ymin);
    map_max_ = QPoint(xmax,ymax);
  }
  
  /**
  @brief Updates the image center by moving directionally
  @param key [int] The key pressed
  @return void
  **/
  void CMapLoader::moveDirectionally(int key)
  {
    int xcenter = (map_max_.x() + map_min_.x()) / 2; 
    int ycenter = (map_max_.y() + map_min_.y()) / 2; 
    QPoint p(xcenter, ycenter);
    //~ float move_by = (map_max_.x() - map_min_.x()) / 10.0;
    float move_by = 50;
    
    switch(key)
    {
      case Qt::Key_Right:
        p.setX(p.x() + move_by);
        break;
      case Qt::Key_Left:
        p.setX(p.x() - move_by);
        break;
      case Qt::Key_Up:
        p.setY(p.y() - move_by);
        break;
      case Qt::Key_Down:
        p.setY(p.y() + move_by);
        break;
      default:
        return;
    }
    updateCenter(p);
  }
  
  
  /**
  @brief Updates the image center
  @param p [QPoint] The new center
  @return void
  **/
  void CMapLoader::updateCenter(QPoint p)
  {
    
    float intW = internal_img_->width();
    float intH = internal_img_->height();
    float newWidth = internal_img_->width() / pow(ZOOM_RATIO,zoom_);
    float newHeight = internal_img_->height() / pow(ZOOM_RATIO,zoom_);
    QPoint evOriginal = p;
    //~ evOriginal.setY(internal_img_->height() - evOriginal.y());
    
    float xmin,xmax,ymin,ymax;
    xmin = evOriginal.x() - newWidth / 2;
    xmax = evOriginal.x() + newWidth / 2;
    ymin = evOriginal.y() - newHeight / 2;
    ymax = evOriginal.y() + newHeight / 2;
    if(xmin < 0)
    {
      xmax += - xmin;
      xmin = 0;
    }
    else if(xmax > internal_img_->width() - 1)
    {
      xmin -= xmax - internal_img_->width() + 1;
      xmax = internal_img_->width() - 1;
    }
    if(ymin < 0)
    {
      ymax += - ymin;
      ymin = 0;
    }
    else if(ymax > internal_img_->height() - 1)
    {
      ymin -= ymax - internal_img_->height() + 1;
      ymax = internal_img_->height() - 1;
    }
    map_min_ = QPoint(xmin,ymin);
    map_max_ = QPoint(xmax,ymax);
  }
  
  /**
  @brief Unscales the input point
  @param p [QPoint] Point of an event in the adjusted map
  @return QPoint : The same point in the original map
  **/
  QPoint CMapLoader::pointUnscaled(QPoint p)
  {
    QPoint newPoint;
    float x = p.x();
    float y = p.y();
    float initialWidth = internal_img_->width();
    float currentWidth = map->width();
    //~ ROS_ERROR("InitialW, CurrW : %f %f",initialWidth,currentWidth);
    float climax = initialWidth / currentWidth;
    newPoint.setX(x * climax);
    newPoint.setY(y * climax);
    return newPoint;
  }
  
  /**
  @brief Resets the zoom of the image
  @return void
  **/
  void CMapLoader::resetZoom(void)
  {
    zoom_ = 0;
    map_min_ = QPoint(0,0);
    map_max_ = QPoint(internal_img_->width() - 1,internal_img_->height() - 1);
  }
    
  /**
  @brief Calculates the "real" point in the image
  @param p [QPoint] The point to be translated
  @return QPoint : The new point
  **/
  QPoint CMapLoader::getGlobalPoint(QPoint p){
    //~ ROS_ERROR("Robot place set (loader) : %d %d",p.x(),p.y());
    QPoint np = pointUnscaled(p);
    //~ ROS_ERROR("Robot place set (unscaled): %d %d",np.x(),np.y());
    int xev = np.x() / pow(ZOOM_RATIO,zoom_) + map_min_.x();
    int yev = np.y() / pow(ZOOM_RATIO,zoom_) + map_min_.y();
    //~ ROS_ERROR("Robot place set (unzoomed): %d %d",xev,yev);
    return QPoint(xev,internal_img_->height() - yev);
  }
  
  /**
  @brief Sets the initial image size
  @param s [QSize] The initial image size
  @return void
  **/
  void CMapLoader::setInitialImageSize(QSize s)
  {
    initial_image_size_ = s;
    map_min_ = QPoint(0, 0);
    map_max_ = QPoint(s.width(), s.height());
  }
}
