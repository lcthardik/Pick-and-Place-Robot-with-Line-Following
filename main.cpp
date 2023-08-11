/*
 * Eyantra_2018_final_code.cpp
 *
 * Created: 23-02-2019 23:38:16
 * Author : dell
 */ 

#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
//#include "color_sensor.h"
#include "buzzer.h"
//#include "sharp.h"
#include "lfr.h"
//#include "My_servo.h"

//int color;
//int distance_sharp;

//void init_devices (void);

void init_devices()
{
	//init_devices_color_sensor(); //colour sensor initalisation
	init_devices_buzzer(); //initiate buzzer port
	//init_devices_sharp_line_sensors();//sharp
	init_devices_lfr(); //line follower
	init_devices_servo();
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
}
/*
  create function for obstacle
  movement functions as per the angles
  change the return angles as per the directions
  connect the obstacle code also
*/
#define inf 9999
#define MAX 10
#define NILL -1
using namespace std;
int prev_x = 0, prev_y = 0;
int current_x = 0, current_y = 0;
int currDir = 1;
int path[30] = {};

const int n = 29; //0 to n-1 nodes
int edges[n][n];
int cost[n][n];
int k = 0;
int last = 0;
int MinDistance(int*, int*);
void PrintPath(int*, int);
void obstacle(int,int,int);

//const int n=25;
  int e = 32; //edges
  int f; //
  int s; //
  int d; //
  int l = 0, m = 0; //
  int newDir;
  int angle;
  int pick_var[10] = {4, 5, 6, 7, 8, 9}; //pick will be same just drop locations will change as per the color sensor
  int color = 1;
  int rdrop[2] = {18, 16};
  int gdrop[2] = {21, 23};
  int bdrop[2] = {27, 28};
  int gd = 0, bd = 0, rd = 0;
  int drop1;
  int sec=0;
int directionArray[8][21] = {
  { 40,  9, 41,   41,  8, 41,   41,  7, 41,   41,  3, 41,   41,  4, 41,   41,  5, 41,   41,  6, 40},

  { 40, 13, 41,   41, 12, 41,   41, 0 , 41,   41,  0, 41,   41,  0, 41,   41, 10, 41,   41, 11, 40},

  { 40,  0, 41,   41,  0, 41,   41, 19, 40,   40,  2, 40,   40, 14, 41,   41,  0, 41,   41,  0, 40},

  { 40,  0, 41,   41,  0, 41,   41,  0, 40,   40,  1, 40,   40,  0, 41,   41,  0, 41,   41,  0, 40},

  { 40, 22, 41,   41, 21, 41,   41, 20, 41,   41,  0, 40,   41, 15, 41,   41, 16, 41,   41, 17, 40},

  { 40, 23, 40,   40,  0, 40,   40,  0, 40,   40, 24, 40,   40,  0, 40,   40,  0, 40,   40, 18, 40},

  { 40,  0, 41,   41,  0, 41,   41,  0, 40,   40, 25, 40,   40,  0, 41,   41,  0, 41,   41,  0, 40},

  { 40,  0, 41,   41,  0, 41,   41, 28, 40,   40, 26, 40,   40, 27, 41,   41,  0, 41,   41,  0, 40},
};
/*
  to print the adjacency matrix
*/
void print(int a[], int n)
{
  for (int i = 0; i < n; i++)
  {
    //Serial.print(a[i]);
    //Serial.print(" ");
  }
}
void reinit(int n)
{
  for (int i = 0; i < n; i++)
  {
    //edges[i]=new int[n];
    for (int j = 0; j < n; j++)
    {
      edges[i][j] = 0;
    }
  }
}
void add_edge(int f, int s)
{
  edges[f][s] = 1;
  edges[s][f] = 1;
}
/*
  to delete the edge if obstacle is there
*/
void rem_edge(int f, int s)
{
  edges[f][s] = 0;
  edges[s][f] = 0;
}
void createGraph()
{
  //add_edge(  0, 1);
  add_edge(  1, 2);
  add_edge(  2, 3);
  //part0
  add_edge(  3, 4);
  add_edge(  4, 5);
  add_edge(  5, 6);
  //part1 0 to 6
  add_edge(  3, 7);
  add_edge(  7, 8);
  add_edge(  8, 9);
  //part2 3 to 9
  add_edge(  8, 12);
  add_edge(  9, 13);
  add_edge(  13, 12);
  //part3
  add_edge(  5, 10);
  add_edge(  6, 11);
  add_edge(  11, 10);// 14
  //part4
  add_edge(  13, 22);
  add_edge(  22, 23);
  add_edge(  22, 21);
  add_edge(  21, 20);
  //part5
  add_edge(  11, 17);
  add_edge(  17, 18);
  add_edge(  17, 16);
  add_edge(  16, 15);
  //part6
  add_edge(  12, 19);
  add_edge(  21, 20);
  add_edge(  19, 20);
  //part7
  add_edge(  10, 14);
  add_edge(  16, 15);
  add_edge(  14, 15);
  //part8

  add_edge(  19, 2);
  add_edge(  14, 2);
  add_edge(  20, 24);
  add_edge(  15, 24);
  add_edge(  24, 25);
  add_edge(  25, 26);
  add_edge(  26, 27);
  add_edge(  26, 28);//36
}
int direct(int prev, int current)
{
  int cflag = 0, pflag = 0;
  //To find the x and y co ordinates of the direction from the matrix
  for (int p = 0; p < 8; p++) {

    for (int m = 0; m < 21; m++)
    {
      if (directionArray[p][m] == current)
      {
        current_y = p;
        current_x = m;
        cflag = 1;
        //cout << "found " << current << " at " << p << " " << m << "\n";
      }
      if (directionArray[p][m] == prev)
      {
        prev_y = p;
        prev_x = m;
        pflag = 1;
        //cout << "found " << current << " at " << p << " " << m << "\n";
      }
      if ((cflag == 1) && (pflag == 1))
      {
        break;
      }
    }
  }//location of the nodes are found
  //direction finder 1-north 5-south 3-east 7-west
  //Serial.println();
  if ((prev_x - current_x) > 0)//upward direction
  {
    if (prev_y == current_y)
    { //Serial.print("West");
      return 7;
    }
    if ((prev_y - current_y) > 0)//left
    { //Serial.print("NW");
      return 8;
    }
    if ((prev_y - current_y) < 0)//right
    { //Serial.print("NE");
      return 6;
    }
  }
  else if ((prev_x - current_x) < 0)//downward direction
  {
    if (prev_y == current_y)
    { //Serial.print("East");
      return 3;
    }
    if ((prev_y - current_y) > 0)//left
    { //Serial.print("SW");
      return 2;
    }
    if ((prev_y - current_y) < 0)//right
    { //Serial.print("SE");
      return 4;
    }
  }
  else if (prev_x == current_x)
  {
    if (prev_y == current_y)
    { //Serial.print("same element at same place ..noiceee");
      return 0;
    }
    if ((prev_y - current_y) > 0)//left
    { //Serial.print("North");
      return 1;
    }
    if ((prev_y - current_y) < 0)//right
    { //Serial.print("South");
      return 5;
    }
  }
  return 0;
}
int traverse(int curr, int next)
{
  //Serial.print("\n Current was ");
  //Serial.print(curr);
  //Serial.print(" Next is ");
  //Serial.print(next);
  //Serial.print(" Here the traversal will take place ");
  currDir = next;
  int diff = 0;
  diff = next - curr;
  /*
    1 - 2 = -1       N - NE
    1 - 3 = -2
    1 - 5 = -4
    1 - 6 = -5
  */
  if (diff > 4)
  {
    return ((diff - 8) * 45);
  }
  if (diff < (-4))
  {
    return ((diff + 8) * 45);
  }
  else
  {
    return (diff * 45);
  }
}
//void dijkstra( int, int , int);
void dijkstra( int _n, int _s, int _d)
{
  int i, u, v, count;
  int dist[n];
  int Blackened[n] = { 0 };
  int pathlength[n] = { 0 };
  int parent[n];
  // The parent Of the source vertex is always equal to nill
  parent[_s] = NILL;
  // first, we initialize all distances to infinity.
  for (i = 0; i < n; i++)
    dist[i] = inf;
  dist[_s] = 0;
  for (count = 0; count < n - 1; count++) {
    u = MinDistance(dist, Blackened);
    // if MinDistance() returns INFINITY, then the graph is not
    // connected and we have traversed all of the vertices in the
    // connected component of the source vertex, so it can reduce
    // the time complexity sometimes
    // In a directed graph, it means that the source vertex
    // is not a root
    if (u == inf)
      break;
    else {
      // Mark the vertex as Blackened
      Blackened[u] = 1;
      for (v = 0; v < n; v++) {
        if (!Blackened[v] && edges[u][v]
            && dist[u] + edges[u][v] < dist[v]) {
          parent[v] = u;
          pathlength[v] = pathlength[parent[v]] + 1;
          dist[v] = dist[u] + edges[u][v];
        }
        else if (!Blackened[v] && edges[u][v]
                 && dist[u] + edges[u][v] == dist[v]
                 && pathlength[u] + 1 < pathlength[v]) {
          parent[v] = u;
          pathlength[v] = pathlength[u] + 1;
        }
      }
    }
  }
  // Printing the path
  if (dist[_d] != inf)
  {
    k = 0;
    PrintPath(parent, _d);
  }
  //else
    //Serial.print("There is no path between vertex ");
  //  << _s << "to vertex " << _d;
}
int MinDistance(int* dist, int* Blackened)
{
  int min = inf, min_index, v;
  for (v = 0; v < n; v++)
    if (!Blackened[v] && dist[v] < min) {
      min = dist[v];
      min_index = v;
    }
  return min == inf ? inf : min_index;
}
// Function to print the path
void PrintPath(int* parent, int _d)
{
  if (parent[_d] == NILL) {
    path[k] = _d;
    k++;
    return;
  }
  PrintPath(parent, parent[_d]);
  path[k] = _d;
  k++;
}

void runs()
{ int newDir;
  int angle;
  angle = 0;

  for (int i = 0; i < k-1 ; i++)
  { //Serial.print("\n At Node ");
    //Serial.println(path[i]);

    if (path[i] == 2 && path[i + 1] == 3)
    {
      obstacle(path[i], path[i + 1], 1);
      i = -1;
      continue;
    }

    if (path[i] == 17 && path[i + 1] == 11)
    {
      obstacle(path[i], path[i + 1], 1);
      i = -1;
      continue;
    }

    if (path[i] == 12 && path[i + 1] == 21)
    {
      obstacle(path[i], path[i + 1], 1);
      i = -1;
      continue;
    }
    newDir = direct(path[i], path[i + 1]);
    angle = traverse(currDir, newDir);
	if (angle == 0)
	{
		forward_by_node(1);
	}
	else if (angle > 0)
	{
		right_mudja();
		forward_by_node(1);
	}
	else if (angle < 0)
	{
		left_mudja();
		forward_by_node(1);
	}
    //Serial.print("Angle ");
    //Serial.print(angle);
  }
}
void obstacle(int prev, int next, int obs)
{
  //Serial.print("There is an obstacle at : ");
  //Serial.print(prev);
  //Serial.print(" ");
  //Serial.println(next);
  rem_edge(prev, next);
  //reverse to previous node
  dijkstra(n, prev, last);
  //Move backwards to the last node
  //currDir=prev;
  //dijkistra(currDir,)
  //remedges(edges,prev,next);
  //direction will be same
  //the path will be found out from reversed node to destination
  //then normal operations will continue
  print(path, n);
}

  void colors(int color)
  {
if(color==1){
  sec=1;       
  if(rd==2)
  sec=3;
  }
else if(color==2){
  sec=2;         
  }
else if(color==3){
  sec=3; 
 }
    
      if (sec == 1)
      {
        drop1 = rdrop[rd];
        rd++;
        ////Serial.println(drop1);
      }
      else if (sec == 2)
      {
        drop1 = gdrop[gd];
        gd++;
      //  //Serial.println(drop1);
        if(gd==2)
            color++;
      }
      else if (sec == 3)
      {
        drop1 = bdrop[bd];
        bd++;
       // //Serial.println(drop1);
        
      }
    }
	
int main() {
	init_devices();
  reinit(n);//empty the whole setup
  //printmat(edges,n);//blank matrix is there
  createGraph();//create the graph
  //printmat(edges,n);//print the new matrix
 
int cls[6]={1,0,1,1,2,2};

  { s = 1;
    for (l = 0; l < 1; l++)
    { 
       
      //Serial.print("\n\n\n Nut ");
      //Serial.print(l + 1);
      last = pick_var[l];
      //Serial.print(" picked : ");
      dijkstra(n, s, pick_var[l]);
      print(path, k);
      runs();
      colors(cls[l]);
      //initialises the value of nuts drop location 
      //returned red=1; blue=2; green=3;
      //before picking it will find the color of the nut
      //here bot will pick the nut


      last = drop1;
      //Serial.print("\n\n\n Dropped : ");
      dijkstra(n, pick_var[l], drop1);
      print(path, k);
      runs();
      //here the bot will drop the nut
      //s=drop1;
    }
    //Serial.print("\n\nOut of for loop");
    //Serial.print("\n\nAll traversal done and returned to home again ");
    //returned back to 1
  _delay_ms(100);
 //exit(1);
    //end of traversal ...beep the siren now beep();
  }
 return 0;

}

/*
int main(void)
{
	init_devices();
    /* Replace with your application code */
   // while (1) 
    //{
		//color=find_color();
		//sound_on();
		
			
		
		//distance_sharp=sharp_sensor_distance();
		//if (distance_sharp < 80 )
		//{
		//	sound_on();
		//}
    //}
	
/*	if(1)
	{
		
	
	forward_by_node(1);
	
	forward_by_node(1);
	
	right_mudja();
	forward_by_node(1);
	stop();
	}
	/*right_mudja();
	forward_by_node(1);
	left_mudja();
	forward_by_node(1);
	left_mudja();
	forward_by_node(1);
	stop();
	_delay_ms(500);
	
	color=find_color();
	
	if (color==1) 
	{
		pick();
		right_mudja();
		forward_by_node(1);
		left_mudja();
		drop();
		stop();
		
	}
	else if (color==2)
	{
		
		left_mudja();
		forward_by_node(1);
		stop();
	}
	else if (color == 3)
	{
		stop();
		sound_on();
	}/*/
	
	
//}





/*
  create function for obstacle
  movement functions as per the angles
  change the return angles as per the directions
  connect the obstacle code also
*/
