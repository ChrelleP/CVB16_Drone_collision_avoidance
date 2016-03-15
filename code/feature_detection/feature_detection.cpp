#include "feature_detection.hpp"

feature_detection::feature_detection()
{
  no_error = true;
}

feature_detection::feature_detection(const char* file)
{
  filename = file;
  no_error = true;
}

void feature_detection::filter(int &lb, int &ub, int &as)
{
  inRange(source, Scalar(lb, lb, lb), Scalar(ub, ub, ub), filtered);

  medianBlur(filtered, filtered, as);
}

void feature_detection::edges(int &lb, int &ub, int &as)
{
  Canny(filtered, edge_map, lb, ub, as);
}

void feature_detection::lines(float &rho, float &theta, int &threshold)
{
  HoughLines(edge_map, hough_lines, rho, theta, threshold, 0, 0);
}

void feature_detection::show_source()
{
  imshow("Source", source);
}

void feature_detection::show_filter()
{
  imshow("Filter", filtered);
}

void feature_detection::show_edge_map()
{
  imshow("Edge map", edge_map);
}

void feature_detection::draw_lines()
{
  for( size_t i = 0; i < hough_lines.size(); i++ )
  {
     float rho = hough_lines[i][0], theta = hough_lines[i][1];
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     line( source, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  }
  for (size_t i = 0; i < hough_lines.size(); i++){
    cout << hough_lines[i][1] << endl;
  }

}

void feature_detection::draw_filtered_lines()
{
  for( size_t i = 0; i < filtered_lines.size(); i++ )
  {
     float rho = filtered_lines[i][0], theta = filtered_lines[i][1];
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     line( source, pt1, pt2, Scalar(0,255,255), 3, CV_AA);
  }
  //cout << "Size of filtered_lines:" << filtered_lines.size() << endl;
}

void feature_detection::draw_objects()
{
  for( size_t i = 0; i < bars.size(); i++ )
  {
     float rho = bars[i].re_center(), theta = bars[i].re_angle();
     cout << "Drawing rho: " << rho << " and theta: " << theta << endl;
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     line( source, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  }
}

void feature_detection::filter_houghlines()
{
  filtered_lines.clear();

  //Normalize the hough lines
  for (size_t i = 0; i < hough_lines.size(); i++) {


    float temp_rho = hough_lines[i][0]/source.cols;
    float temp_theta = hough_lines[i][1]/(M_PI);

    bool replace = false;
    bool wrap_forward = false;
    bool wrap_reverse = false;

    //cout << "Temp rho: " << temp_rho << " temp theta: " << temp_theta << endl;
    if (temp_theta > ((175 * (M_PI / 180)) / M_PI)) { // degrees to radians check
      wrap_forward = true;
      //cout << "wrap_forward"  << endl;
    }
    if (temp_theta < ((5 * (M_PI / 180)) / M_PI) ) {
      wrap_reverse = true;
      //cout << "wrap_reverse" << endl;
    }

    for(int i = 0; i < filtered_lines.size(); i++){
      Vec2f filtered_line = filtered_lines[i];
      if (wrap_reverse) {
        if(filtered_line[1] > ((135 * (M_PI / 180)) / (M_PI))){ // degrees to radians check
          filtered_line[1] -= 1;
        }
      }
      if (wrap_forward) {
        if(filtered_line[1] < ((45 * (M_PI / 180)) / (M_PI))){ // degrees to radians check
          filtered_line[1] += 1;
        }
      }


      // Negative rho check
      if (signbit(filtered_line[0]) && signbit(temp_rho)){
        // If both negative
        }
        else{
        // If both not negative
        }

      float temp_dst = dst_hspace(Vec2f(temp_rho, temp_theta), filtered_line);
      //cout << "Filter rho: " << filtered_line[0] << " theta: " << filtered_line[1] << endl;
      //cout << "Temp_dst: " << temp_dst << endl;
      //cout << endl;


      if(temp_dst < 0.015){
        // Negative rho check, correct temp_line assignment.
        if (signbit(filtered_line[0]) && signbit(temp_rho)){
          // If both negative
          Vec2f temp_line((temp_rho + filtered_line[0])/2, (temp_theta + filtered_line[1])/2 );
          filtered_lines[i] = temp_line;
          }
          else{
          // If both not negative
          Vec2f temp_line( (fabs(temp_rho) + fabs(filtered_line[0]))/2, (temp_theta + filtered_line[1])/2 );
          filtered_lines[i] = temp_line;
          }
        replace = true;
      }
    }

    //Append the temp values to filtered_lines
    if(!replace)
      filtered_lines.push_back(Vec2f(temp_rho, temp_theta));
  }

  // Denormalize
  for(int i = 0; i < filtered_lines.size(); i++)
  {
    filtered_lines[i][0] *= source.cols;
    filtered_lines[i][1] *= M_PI;
  }
}

float feature_detection::dst_hspace(Vec2f first, Vec2f second)
{
  // The distance calculation is calculated with fabs() rho, since theres is no need for negative distances in Hough Space.
  float temp = sqrt( powf(fabs(first[0])-fabs(second[0]), 2) + powf(first[1]-second[1], 2) );
  return temp;
}

void feature_detection::identify_objects()
{
  for(int i = 0; i < filtered_lines.size(); i++)
  {
    bool updated = false;

    for(int k = 0; k < bars.size(); k++)
    {
        if( bars[k].update(filtered_lines[i]) )
        {
          updated = true;
          cout << "Updated" << endl;
          break;
        }
    }

    if(!updated)
    {
      for(int j = 0; j < filtered_lines.size(); j++)
      {
        if(i != j)
        {
          float temp_total_angle = (filtered_lines[i][1] - filtered_lines[j][1]);
          if( temp_total_angle < 0.087 && temp_total_angle > -0.087 ) // 5 degrees
          {
            object temp(filtered_lines[i], filtered_lines[j]);
            bars.push_back(temp);
          }
        }
      }
     }

     cout << "Bars: " << bars.size() << endl;

     for(int l = 0; l < bars.size(); l++)
     {
       bool temp = bars[l].alive_decount();
       if(temp)
       {
         if(l > 0)
         {
           bars.erase(bars.begin()+l);
           cout << "erasing" << endl;
         }
         else
         {
           bars.erase(bars.begin());
           cout << "erasing" << endl;
         }
       }
     }

  }
}

feature_detection::~feature_detection()
{

}
