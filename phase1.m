#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

typedef struct 
{
  int x; /* x-coordinate of sensor node location in target field */
  int y; /* y-coordinate of sensor node location in target field */
  int *keyring; /* key ring */
 
  int phynbrsize; /* number of physical neighbors of a sensor node */
  int *phynbr; /* List of physical neighbors of a sensor node */
 
  int keynbrsize; /* number of key neighbors of a sensor node */
  int *keynbr; /* List of key neighbors of a sensor node */
   
} sensor;

sensor sensor_node[100000];
float total_path_key_one_hop_count =0.0;
float total_path_key_two_hop_count =0.0;
float total_key=0.0, total_physical=0.0;

double euclidean_distance(int x1, int y1, int x2, int y2)
{  
  double diffx = x1 - x2;
  double diffy = y1 - y2;
  double diffx_sqr = pow(diffx, 2);
  double diffy_sqr = pow(diffy, 2);
  double distance = sqrt (diffx_sqr + diffy_sqr);
  return distance;
}

void finding_network_connectivity (int n, int key_ring_size, int key_pool_size, float average_neighbour_size)
{ 
  printf("\n");
   printf("Network Connectivities : \n");
 int i, count=0;
 float num_one_hop, num_two_hop;
 float p_0, p_1, p_2;
 for(i=0; i<n; i++)
        {
          total_key = total_key + sensor_node[i].keynbrsize;
          total_physical = total_physical + sensor_node[i].phynbrsize; 
        }
printf("\n");
 printf("Simulated Average Network Connectivity\n");
 printf("%f\n", total_key/total_physical);

 printf("Theoritical Network Connectivity\n");
 float pro=1.0, num, den, peg;
 for (i=0; i<key_ring_size ;i++)
      {
        num = key_pool_size - key_ring_size - i;
        den = key_pool_size - i;
        pro = pro * (num/den);
      }
  p_0 = 1-pro;
  printf("%f\n", p_0);


  num_one_hop = (total_path_key_one_hop_count + total_key);
  num_two_hop = (total_path_key_one_hop_count + total_path_key_two_hop_count + total_key);
  p_1 = pow((1-pow(p_0,2)), average_neighbour_size);
  p_1 = (1 - p_0) * p_1;
  p_1 = 1 - p_1;
  printf("Simulated Average Network Connectivity for one hop\n");
  printf("%f\n", num_one_hop/total_physical);
  printf("Theoritical Average Network Connectivity for one hop\n");
  printf("%f\n", p_1 );
  p_2 = pow((1-(p_0 * p_1)), average_neighbour_size);
  p_2 = p_2 * (1-p_1);
  p_2 = 1 - p_2;
  printf("Simulated Average Network Connectivity for two hop\n");
  printf("%f\n", num_two_hop/total_physical);
  printf("Theoritical Average Network Connectivity for two hop\n");
  printf("%f\n", p_2 );
  


}

void simulating_path_key_establishment(int n, int key_ring_size, int key_pool_size)
{
  printf("Computing key neighborhood in direct key establishment phase...\n");
  printf("Computing key neighborhood in path key establishment phase...\n");

  int i, j , k, x, y, index, z, w;
  int arr[10000], temp_physical[1000], one_hop[1000], one_hop_temp[1000];
  for(i=0; i<n; i++)
      {
        index = 0;
        for(x=0; x<sensor_node[i].phynbrsize; x++)
                {
                  arr[index] = *(sensor_node[i].phynbr+x);
                  index++;
                }
        for(j=0; j<sensor_node[i].keynbrsize; j++)
              {
                for(k=0; k<sensor_node[i].phynbrsize; k++)
                    {
                      if( *(sensor_node[i].keynbr+j)==arr[k])
                          arr[k]=-1;
                    }
              }
        int t=0,flag=0;
        for(z=0; z<index; z++)
              {
                if(arr[z]!=-1)
                    {
                      temp_physical[t] = arr[z];
                      t++;
                    }
              }

        /* One Hop Calculation */
        int one_hop_index = 0;
        for(x=0; x<t; x++)
            {
            flag=0;
             for(y=0; y<sensor_node[i].keynbrsize; y++)
                  {
                    for(z=0; z<sensor_node[*(sensor_node[i].keynbr+y)].keynbrsize; z++)
                          {
                            if((*(sensor_node[*(sensor_node[i].keynbr+y)].keynbr+z)) == temp_physical[x])
                                {
                                  total_path_key_one_hop_count++;
                                  one_hop[one_hop_index]= temp_physical[x];
                                  one_hop_index++;
                                  flag=1;
                                  break;
                                }
                          }
                    if(flag==1)
                      break;      
                    
                  }
            }
        int one_t=0; 

        /* Calculation of two hop temporary physical array */
        for(x=0; x<one_hop_index; x++)
              {
                for(y=0; y<t; y++)
                    {
                      if(one_hop[x]==temp_physical[y])
                          temp_physical[y]=-1;
                    }
              }
         for(x=0; x<t; x++)
              {
                if(temp_physical[x]!=-1)
                      {
                        one_hop_temp[one_t]=temp_physical[x];
                        one_t++;
                      }
              }

        /* Calculation of two hop counts */
        for(x=0; x<one_t; x++)
            {
            flag=0;
             for(y=0; y<sensor_node[i].keynbrsize; y++)
                  {
                    for(z=0; z<sensor_node[*(sensor_node[i].keynbr+y)].keynbrsize; z++)
                          {
                            
                            for(w=0; w<sensor_node[*(sensor_node[*(sensor_node[i].keynbr+y)].keynbr+z)].keynbrsize; w++)
                                {
                                  if((*(sensor_node[*(sensor_node[*(sensor_node[i].keynbr+y)].keynbr+z)].keynbr+w)) == one_hop_temp[x])
                                    {
                                      total_path_key_two_hop_count++;
                                      flag=1;
                                      break;
                                    }
                                if(flag==1)
                                    break;      
                    
                                }
                            if(flag==1)
                                    break;   
                          }
                    if(flag==1)
                      break;      
                    
                  }
            }
      
       }


}
void printing_sensor_nodes (int n, int key_ring_size, int key_pool_size)
{
  int i, x;
 printf("Printing Structure\n");
    for(i=0; i<n; i++)
        {
          printf("NODE %d\n", i);
          printf("%d %d\n", sensor_node[i].x, sensor_node[i].y );
          printf("Physical neighbors size %d\n", sensor_node[i].phynbrsize);
          for(x=0; x<sensor_node[i].phynbrsize; x++)
                {
                  printf("%d ", *(sensor_node[i].phynbr+x) );
                }
          printf("\n");

           printf("Key neighbors size %d\n", sensor_node[i].keynbrsize);
          for(x=0; x<sensor_node[i].keynbrsize; x++)
                {
                  printf("%d ", *(sensor_node[i].keynbr+x) );
                }
          printf("\n");
        }

}
float finding_physical_neighbours(int n, int key_ring_size, int key_pool_size)
{
   int i, j, x, k;
   double distance;
   double total_distance=0.0;
   double physical_neighbour_size=0.0;
   float average_neighbour_size=0.0;
   for(i=0; i<n; i++)
        {
          sensor_node[i].phynbrsize=0;
          int phy_neighbours_array[100000];   
          for(j=0; j<n; j++)
              {
              if(i!=j)
                    {
                      distance = euclidean_distance(sensor_node[i].x, sensor_node[i].y, sensor_node[j].x, sensor_node[j].y);
                      if (i<j)
                        total_distance = total_distance + distance;
                      if(distance<25.00)
                              {
                                phy_neighbours_array[sensor_node[i].phynbrsize] = j;
                                sensor_node[i].phynbrsize++;       
                              }
                    }
              }

         //physical_neighbour_size = physical_neighbour_size + sensor_node[i].phynbrsize;
         sensor_node[i].phynbr  =(int*)malloc(sensor_node[i].phynbrsize*sizeof(int));
         for(k=0; k<sensor_node[i].phynbrsize; k++)
                {
                  *(sensor_node[i].phynbr+k) = phy_neighbours_array[k];
                }
        } 
      printf("Scaling communication range...\n"); 
    printf("Average distance = %f\n", total_distance/((n-1)*(n/2)) );
    printf("Communication range of sensor nodes = 25.00\n");
    printf("\n");
    printf("Computing physical neighbors...\n");

    for(i=0; i<n; i++)
          {
            physical_neighbour_size = physical_neighbour_size + sensor_node[i].phynbrsize;
          }
   // printf("%f\n", physical_neighbour_size );

    average_neighbour_size = physical_neighbour_size/n;
    printf("Average neighborhood size = %f\n", average_neighbour_size  );
    return average_neighbour_size;
}

void finding_key_neighbours (int n, int key_ring_size, int key_pool_size)
{
   int i, j, x, k, y,z;
   printf("\n");
   printf("EG scheme\n");
   printf("Distributing keys...\n");
   printf("\n");
   double distance;
   int flag=0;
   int key_neighbours_array[100000]; 
   for(i=0; i<n; i++)
        {
          sensor_node[i].keynbrsize=0;
          for(x=0; x<sensor_node[i].phynbrsize; x++)
                {
                  flag=0;
                  int physical_neighbour_id = *(sensor_node[i].phynbr+x);
                  for(y=0 ; y<key_ring_size; y++)
                          {
                            for(z=0; z<key_ring_size; z++)
                                    {
                                      if(*(sensor_node[i].keyring+y) == *(sensor_node[physical_neighbour_id].keyring+z) )
                                              {
                                              
                                                flag=1;
                                                key_neighbours_array[sensor_node[i].keynbrsize] = physical_neighbour_id;
                                                sensor_node[i].keynbrsize++;
                                               break;
                                              }
                                    }
                            if(flag==1) break;
                          }
                }
          sensor_node[i].keynbr   =(int*)malloc(sensor_node[i].keynbrsize*sizeof(int));
          for(k=0; k<sensor_node[i].keynbrsize; k++)
                {
                  *(sensor_node[i].keynbr+k) = key_neighbours_array[k];
                }

          
        }
}

void generating_random_numbers(int n, FILE *fp2, int key_ring_size, int key_pool_size)
{
  time_t t;
  char str[15];
  int i, j, x, y, index, n1, n2;
  srand((unsigned) time(&t));
  sprintf(str, "%d", n);
  fprintf(fp2,"%s\n", str);
  int flag=0, key1;
  int fkey_ring=0;


  //allocation of x-y values and keyring
  for(i=0; i<n; i++)
            {
            sensor_node[i].keyring =(int*)malloc(key_ring_size*sizeof(int));
            for(x=0; x<key_ring_size; x++)
                {
                  fkey_ring=0;
                  key1 = rand() % key_pool_size + 1;
                  for(y=0; y<x; y++)
                      {
                      if (*(sensor_node[i].keyring + y) == key1)
                            {
                              x--;
                              fkey_ring=1;
                              break;
                            }    
                      }
                  if(fkey_ring!=1)
                      *(sensor_node[i].keyring + x) = key1;
                }
            flag=0;
            n1 = rand() % 500 + 1;
            n2 = rand() % 500 + 1; 
            for(j=0; j<i; j++)
                    {
                      if(sensor_node[j].x == n1 && sensor_node[j].y == n2)
                              {
                                flag=1;
                                i--;
                                break;
                              }
                    }
            if(flag!=1)
                  {
                      index++;
                      sensor_node[i].x = n1;
                      sensor_node[i].y = n2;
                      sprintf(str, "%d", n1);
                      fprintf(fp2,"%s ", str);
                      sprintf(str, "%d", n2);
                      fprintf(fp2,"%s\n", str);
                  }           
            
            }
  fclose(fp2);
}
int main( int argc, char *argv[] )  
{
    int L1 = 500, L2=500, n;  
    float average_neighbour_size; 
    if(argc!=4)
        {
          printf("No of arguments passes are invalid\n");
          printf("Usage :  ./a.out DATA-FILE KEYRING-SIZE KEYPOOL-SIZE");
          return 0;
        }
    printf("Enter the no of sensor nodes\n");
    scanf("%d", &n);
    FILE *fp2;
    fp2=fopen(argv[1], "w");
    if(fp2==NULL) 
      {
        perror("Error opening file.");
      }
    FILE *fp1;
    fp1=fopen("name.gp", "w");
    char string[10000] = "plot[0:500][0:500] \"";
    strcat(string, argv[1]);
    strcat(string, "\" every ::1 using 1:2");
    fprintf(fp1,"%s", string);
    fclose(fp1);  
    generating_random_numbers(n, fp2, atoi(argv[2]), atoi(argv[3]));
    average_neighbour_size = finding_physical_neighbours(n, atoi(argv[2]), atoi(argv[3]));
    finding_key_neighbours(n, atoi(argv[2]), atoi(argv[3]));
   // printing_sensor_nodes(n, atoi(argv[2]), atoi(argv[3]));
    simulating_path_key_establishment(n, atoi(argv[2]), atoi(argv[3]));
    finding_network_connectivity(n, atoi(argv[2]), atoi(argv[3]), average_neighbour_size);
    system("gnuplot -p 'name.gp'"); 
    
    return 0;
}
typedef struct 
{
  int x; /* x-coordinate of sensor node location in target field */
  int y; /* y-coordinate of sensor node location in target field */
  int *keyring; /* key ring */
 
  int phynbrsize; /* number of physical neighbors of a sensor node */
  int *phynbr; /* List of physical neighbors of a sensor node */
 
  int keynbrsize; /* number of key neighbors of a sensor node */
  int *keynbr; /* List of key neighbors of a sensor node */
   
} sensor;

sensor sensor_node[100000];
float total_path_key_one_hop_count =0.0;
float total_path_key_two_hop_count =0.0;
float total_key=0.0, total_physical=0.0;

double euclidean_distance(int x1, int y1, int x2, int y2)
{  
  double diffx = x1 - x2;
  double diffy = y1 - y2;
  double diffx_sqr = pow(diffx, 2);
  double diffy_sqr = pow(diffy, 2);
  double distance = sqrt (diffx_sqr + diffy_sqr);
  return distance;
}

void finding_network_connectivity (int n, int key_ring_size, int key_pool_size, float average_neighbour_size)
{ 
  printf("\n");
   printf("Network Connectivities : \n");
 int i, count=0;
 float num_one_hop, num_two_hop;
 float p_0, p_1, p_2;
 for(i=0; i<n; i++)
        {
          total_key = total_key + sensor_node[i].keynbrsize;
          total_physical = total_physical + sensor_node[i].phynbrsize; 
        }
printf("\n");
 printf("Simulated Average Network Connectivity\n");
 printf("%f\n", total_key/total_physical);

 printf("Theoritical Network Connectivity\n");
 float pro=1.0, num, den, peg;
 for (i=0; i<key_ring_size ;i++)
      {
        num = key_pool_size - key_ring_size - i;
        den = key_pool_size - i;
        pro = pro * (num/den);
      }
  p_0 = 1-pro;
  printf("%f\n", p_0);


  num_one_hop = (total_path_key_one_hop_count + total_key);
  num_two_hop = (total_path_key_one_hop_count + total_path_key_two_hop_count + total_key);
  p_1 = pow((1-pow(p_0,2)), average_neighbour_size);
  p_1 = (1 - p_0) * p_1;
  p_1 = 1 - p_1;
  printf("Simulated Average Network Connectivity for one hop\n");
  printf("%f\n", num_one_hop/total_physical);
  printf("Theoritical Average Network Connectivity for one hop\n");
  printf("%f\n", p_1 );
  p_2 = pow((1-(p_0 * p_1)), average_neighbour_size);
  p_2 = p_2 * (1-p_1);
  p_2 = 1 - p_2;
  printf("Simulated Average Network Connectivity for two hop\n");
  printf("%f\n", num_two_hop/total_physical);
  printf("Theoritical Average Network Connectivity for two hop\n");
  printf("%f\n", p_2 );
  


}

void simulating_path_key_establishment(int n, int key_ring_size, int key_pool_size)
{
  printf("Computing key neighborhood in direct key establishment phase...\n");
  printf("Computing key neighborhood in path key establishment phase...\n");

  int i, j , k, x, y, index, z, w;
  int arr[10000], temp_physical[1000], one_hop[1000], one_hop_temp[1000];
  for(i=0; i<n; i++)
      {
        index = 0;
        for(x=0; x<sensor_node[i].phynbrsize; x++)
                {
                  arr[index] = *(sensor_node[i].phynbr+x);
                  index++;
                }
        for(j=0; j<sensor_node[i].keynbrsize; j++)
              {
                for(k=0; k<sensor_node[i].phynbrsize; k++)
                    {
                      if( *(sensor_node[i].keynbr+j)==arr[k])
                          arr[k]=-1;
                    }
              }
        int t=0,flag=0;
        for(z=0; z<index; z++)
              {
                if(arr[z]!=-1)
                    {
                      temp_physical[t] = arr[z];
                      t++;
                    }
              }

        /* One Hop Calculation */
        int one_hop_index = 0;
        for(x=0; x<t; x++)
            {
            flag=0;
             for(y=0; y<sensor_node[i].keynbrsize; y++)
                  {
                    for(z=0; z<sensor_node[*(sensor_node[i].keynbr+y)].keynbrsize; z++)
                          {
                            if((*(sensor_node[*(sensor_node[i].keynbr+y)].keynbr+z)) == temp_physical[x])
                                {
                                  total_path_key_one_hop_count++;
                                  one_hop[one_hop_index]= temp_physical[x];
                                  one_hop_index++;
                                  flag=1;
                                  break;
                                }
                          }
                    if(flag==1)
                      break;      
                    
                  }
            }
        int one_t=0; 

        /* Calculation of two hop temporary physical array */
        for(x=0; x<one_hop_index; x++)
              {
                for(y=0; y<t; y++)
                    {
                      if(one_hop[x]==temp_physical[y])
                          temp_physical[y]=-1;
                    }
              }
         for(x=0; x<t; x++)
              {
                if(temp_physical[x]!=-1)
                      {
                        one_hop_temp[one_t]=temp_physical[x];
                        one_t++;
                      }
              }

        /* Calculation of two hop counts */
        for(x=0; x<one_t; x++)
            {
            flag=0;
             for(y=0; y<sensor_node[i].keynbrsize; y++)
                  {
                    for(z=0; z<sensor_node[*(sensor_node[i].keynbr+y)].keynbrsize; z++)
                          {
                            
                            for(w=0; w<sensor_node[*(sensor_node[*(sensor_node[i].keynbr+y)].keynbr+z)].keynbrsize; w++)
                                {
                                  if((*(sensor_node[*(sensor_node[*(sensor_node[i].keynbr+y)].keynbr+z)].keynbr+w)) == one_hop_temp[x])
                                    {
                                      total_path_key_two_hop_count++;
                                      flag=1;
                                      break;
                                    }
                                if(flag==1)
                                    break;      
                    
                                }
                            if(flag==1)
                                    break;   
                          }
                    if(flag==1)
                      break;      
                    
                  }
            }
      
       }


}
void printing_sensor_nodes (int n, int key_ring_size, int key_pool_size)
{
  int i, x;
 printf("Printing Structure\n");
    for(i=0; i<n; i++)
        {
          printf("NODE %d\n", i);
          printf("%d %d\n", sensor_node[i].x, sensor_node[i].y );
          printf("Physical neighbors size %d\n", sensor_node[i].phynbrsize);
          for(x=0; x<sensor_node[i].phynbrsize; x++)
                {
                  printf("%d ", *(sensor_node[i].phynbr+x) );
                }
          printf("\n");

           printf("Key neighbors size %d\n", sensor_node[i].keynbrsize);
          for(x=0; x<sensor_node[i].keynbrsize; x++)
                {
                  printf("%d ", *(sensor_node[i].keynbr+x) );
                }
          printf("\n");
        }

}
float finding_physical_neighbours(int n, int key_ring_size, int key_pool_size)
{
   int i, j, x, k;
   double distance;
   double total_distance=0.0;
   double physical_neighbour_size=0.0;
   float average_neighbour_size=0.0;
   for(i=0; i<n; i++)
        {
          sensor_node[i].phynbrsize=0;
          int phy_neighbours_array[100000];   
          for(j=0; j<n; j++)
              {
              if(i!=j)
                    {
                      distance = euclidean_distance(sensor_node[i].x, sensor_node[i].y, sensor_node[j].x, sensor_node[j].y);
                      if (i<j)
                        total_distance = total_distance + distance;
                      if(distance<25.00)
                              {
                                phy_neighbours_array[sensor_node[i].phynbrsize] = j;
                                sensor_node[i].phynbrsize++;       
                              }
                    }
              }

         //physical_neighbour_size = physical_neighbour_size + sensor_node[i].phynbrsize;
         sensor_node[i].phynbr  =(int*)malloc(sensor_node[i].phynbrsize*sizeof(int));
         for(k=0; k<sensor_node[i].phynbrsize; k++)
                {
                  *(sensor_node[i].phynbr+k) = phy_neighbours_array[k];
                }
        } 
      printf("Scaling communication range...\n"); 
    printf("Average distance = %f\n", total_distance/((n-1)*(n/2)) );
    printf("Communication range of sensor nodes = 25.00\n");
    printf("\n");
    printf("Computing physical neighbors...\n");

    for(i=0; i<n; i++)
          {
            physical_neighbour_size = physical_neighbour_size + sensor_node[i].phynbrsize;
          }
   // printf("%f\n", physical_neighbour_size );

    average_neighbour_size = physical_neighbour_size/n;
    printf("Average neighborhood size = %f\n", average_neighbour_size  );
    return average_neighbour_size;
}

void finding_key_neighbours (int n, int key_ring_size, int key_pool_size)
{
   int i, j, x, k, y,z;
   printf("\n");
   printf("EG scheme\n");
   printf("Distributing keys...\n");
   printf("\n");
   double distance;
   int flag=0;
   int key_neighbours_array[100000]; 
   for(i=0; i<n; i++)
        {
          sensor_node[i].keynbrsize=0;
          for(x=0; x<sensor_node[i].phynbrsize; x++)
                {
                  flag=0;
                  int physical_neighbour_id = *(sensor_node[i].phynbr+x);
                  for(y=0 ; y<key_ring_size; y++)
                          {
                            for(z=0; z<key_ring_size; z++)
                                    {
                                      if(*(sensor_node[i].keyring+y) == *(sensor_node[physical_neighbour_id].keyring+z) )
                                              {
                                              
                                                flag=1;
                                                key_neighbours_array[sensor_node[i].keynbrsize] = physical_neighbour_id;
                                                sensor_node[i].keynbrsize++;
                                               break;
                                              }
                                    }
                            if(flag==1) break;
                          }
                }
          sensor_node[i].keynbr   =(int*)malloc(sensor_node[i].keynbrsize*sizeof(int));
          for(k=0; k<sensor_node[i].keynbrsize; k++)
                {
                  *(sensor_node[i].keynbr+k) = key_neighbours_array[k];
                }

          
        }
}

void generating_random_numbers(int n, FILE *fp2, int key_ring_size, int key_pool_size)
{
  time_t t;
  char str[15];
  int i, j, x, y, index, n1, n2;
  srand((unsigned) time(&t));
  sprintf(str, "%d", n);
  fprintf(fp2,"%s\n", str);
  int flag=0, key1;
  int fkey_ring=0;


  //allocation of x-y values and keyring
  for(i=0; i<n; i++)
            {
            sensor_node[i].keyring =(int*)malloc(key_ring_size*sizeof(int));
            for(x=0; x<key_ring_size; x++)
                {
                  fkey_ring=0;
                  key1 = rand() % key_pool_size + 1;
                  for(y=0; y<x; y++)
                      {
                      if (*(sensor_node[i].keyring + y) == key1)
                            {
                              x--;
                              fkey_ring=1;
                              break;
                            }    
                      }
                  if(fkey_ring!=1)
                      *(sensor_node[i].keyring + x) = key1;
                }
            flag=0;
            n1 = rand() % 500 + 1;
            n2 = rand() % 500 + 1; 
            for(j=0; j<i; j++)
                    {
                      if(sensor_node[j].x == n1 && sensor_node[j].y == n2)
                              {
                                flag=1;
                                i--;
                                break;
                              }
                    }
            if(flag!=1)
                  {
                      index++;
                      sensor_node[i].x = n1;
                      sensor_node[i].y = n2;
                      sprintf(str, "%d", n1);
                      fprintf(fp2,"%s ", str);
                      sprintf(str, "%d", n2);
                      fprintf(fp2,"%s\n", str);
                  }           
            
            }
  fclose(fp2);
}
int main( int argc, char *argv[] )  
{
    int L1 = 500, L2=500, n;  
    float average_neighbour_size; 
    if(argc!=4)
        {
          printf("No of arguments passes are invalid\n");
          printf("Usage :  ./a.out DATA-FILE KEYRING-SIZE KEYPOOL-SIZE");
          return 0;
        }
    printf("Enter the no of sensor nodes\n");
    scanf("%d", &n);
    FILE *fp2;
    fp2=fopen(argv[1], "w");
    if(fp2==NULL) 
      {
        perror("Error opening file.");
      }
    FILE *fp1;
    fp1=fopen("name.gp", "w");
    char string[10000] = "plot[0:500][0:500] \"";
    strcat(string, argv[1]);
    strcat(string, "\" every ::1 using 1:2");
    fprintf(fp1,"%s", string);
    fclose(fp1);  
    generating_random_numbers(n, fp2, atoi(argv[2]), atoi(argv[3]));
    average_neighbour_size = finding_physical_neighbours(n, atoi(argv[2]), atoi(argv[3]));
    finding_key_neighbours(n, atoi(argv[2]), atoi(argv[3]));
   // printing_sensor_nodes(n, atoi(argv[2]), atoi(argv[3]));
    simulating_path_key_establishment(n, atoi(argv[2]), atoi(argv[3]));
    finding_network_connectivity(n, atoi(argv[2]), atoi(argv[3]), average_neighbour_size);
    system("gnuplot -p 'name.gp'"); 
    
    return 0;
}