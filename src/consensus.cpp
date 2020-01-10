//
// Created by miguel on 03/12/19.
//

#include <stdio.h>
#include "../include/consensus.h"
#include <math.h>

c_node::c_node(int lenght_t, int id_in_array, float* KARR, float own_c,float own_o, float own_L, float rho_t)
{
    lenght = lenght_t;
    index = id_in_array;
    d = new float[lenght];
    for(int i = 0; i < lenght; i++) d[i] = 0;
    d_av = new float[lenght];
    for(int i = 0; i < lenght; i++) d_av[i] = 0;
    y = new float[lenght];
    for(int i = 0; i< lenght; i++) y[i] = 0;
    k = KARR;
    n = 0;
    for(int i = 0; i < lenght; i++) n = n + (float) pow(k[i],2);
    m = n - (float)pow(k[id_in_array], 2);
    c = new float[lenght];
    for(int i = 0; i < lenght; i++) c[i] = 0;
    c[id_in_array] = own_c;
    o = own_o;
    L = own_L;
    rho = rho_t;
    next_it = new float[lenght];
    cost_it = 0;
}



float c_node::evaluate_cost(float d_aux[])
{

    float cost;
    auto* diff1 = new float[lenght];

    VectDifference(d_aux, d_av, diff1);

    cost = dotProduct(c, d_aux) + dotProduct(y, diff1) + rho/2*(sum_of_squares(diff1));

    delete[] diff1;

    return cost;
}

// Function that returns
// the dot product of two vector array.
float c_node::dotProduct(const float vectA[], const float vectB[]) //TESTED
{
    float product = 0;

    // Loop for calculate cot product
    for (int i = 0; i < lenght; i++)
        product = product + vectA[i] * vectB[i];
    return product;
}

void c_node::VectDifference(float vectA[], float vectB[], float diff[]) // TESTED
{

    for (int i = 0; i < lenght; i++){
        diff[i] = vectA[i] - vectB[i];
    }

}



bool c_node::check_feasibility(float d_aux[]) //TESTED
{
    float tol = 0.001;
    bool check = true;
   
    if (d_aux[index] < 0-tol)
    {
        check = false;
        return check;
    }

    if (d_aux[index] > 100 + tol)
    {
        check = false;
        return check;
    }
   
      
    if (dotProduct(d_aux, k) < L - o - tol)
    {
        check = false;
      return check;
    }

    return check;

}
float c_node::sum_of_squares(float vect[]) //TESTED
{
    float sum = 0;
    for(int i = 0; i<lenght; i++)
    {
        sum = sum + (float)pow(vect[i],2);
    }
    return sum;
}
void c_node::update_average(float values[], int index)
{
    float temp = 0;
    for(int i = 0; i<lenght; i++)
    {
        temp = temp + values[i];
    }
    d_av[index] = temp/(float)lenght;
}

void c_node::update_lagrange()
{
    for(int i = 0; i < lenght; i++)
    {
      y[i] = y[i] + rho*(d[i] - d_av[i]);
    }
}

void c_node::consensus_iterate()
{

    int i = 0;
    float cost;
    float z[lenght], d_u[lenght], d_bli[lenght];
    float d_b0[lenght], d_b1[lenght], d_l0[lenght], d_l1[lenght], d_aux = 0;

    cost_it = 100000;

  for(i = 0; i < lenght; i++) next_it[i] = -1;

  //CHECK D_U
  for(i = 0; i < lenght; i++)
  {
    z[i] = d_av[i]*rho - y[i]- c[i];
    d_u[i] = z[i]/rho;
  }

  if(check_feasibility(d_u))
  {
    cost = evaluate_cost(d_u);
    if(cost < cost_it)
      for ( i = 0; i < lenght; i++)
      {
        next_it[i] = d_u[i];
      }
    cost_it= cost;
  }
  //END OF CHECK D_U
  d_aux = dotProduct(z,k);
  //CHECK D_BLI
  for ( i = 0; i < lenght; i++) d_bli[i] = (1/rho)*z[i] - k[i]/n*(o-L+(1/rho)*d_aux);

  if(check_feasibility(d_bli))
  {
    cost = evaluate_cost(d_bli);
    if (cost < cost_it)
    {
      for ( i = 0; i < lenght; i++)
      {
        next_it[i] = d_bli[i];
      }
      cost_it = cost;
    }
  }
  //END OF CHECK D_BLI
  //CHECK D_B0
  for(i = 0; i < lenght; i++)
  {
      d_b0[i] = z[i]/rho;
  }
  d_b0[index] = 0;
  if(check_feasibility(d_b0))
  {
    cost = evaluate_cost(d_b0);
    if(cost < cost_it)
    {
      for ( i = 0; i < lenght; i++)
      {
        next_it[i] = d_b0[i];
      }
      cost_it= cost;
    }
  }
//END OF CHECK D_B0
//CHECK D_B1

  for(i = 0; i < lenght; i++)
  {
    d_b1[i] = z[i]/rho;
  }
  d_b1[index] = 100;
  if(check_feasibility(d_b1))
  {
    cost = evaluate_cost(d_b1);
    if(cost < cost_it)
    {
      for ( i = 0; i < lenght; i++)
      {
        next_it[i] = d_b1[i];
      }
      cost_it = cost;
    }
  }
  //END CHECK D_B1
  //CHECK D_L0

  for ( i = 0; i < lenght; i++)
  {
      d_l0[i] = (1/rho)*z[i] - (1/m)*k[i]*(o-L) + (1/rho/m)*k[i]*(k[index]*z[index]-d_aux);

  }
  d_l0[index] = 0;
  if(check_feasibility(d_l0))
  {
    cost = evaluate_cost(d_l0);
    if(cost < cost_it)
    {
      for ( i = 0; i < lenght; i++)
      {
        next_it[i] = d_l0[i];
      }
      cost_it = cost;
    }
  }
//END CHECK D_L0
//CHECK D_L1
  for ( i = 0; i < lenght; i++)
  {
      d_l1[i] = (1/rho)*z[i] - (1/m)*k[i]*(o-L+100*k[index]) + (1/rho/m)*k[i]*(k[index]*z[index]-d_aux);
  }
  d_l1[index] = 100;
    if(check_feasibility(d_l1))
    {
        cost = evaluate_cost(d_l1);
        if(cost < cost_it)
        {
            for ( i = 0; i < lenght; i++)
            {
                next_it[i] = d_l1[i];
            }
            cost_it = cost;
        }
    }
    for(i = 0; i < lenght; i++){
      d[i] = next_it[i];
//      printf(" d_l0[%d]: %f || d_b1[%d]: %f || d_bl0[%d] %f || d_bli[%d] %f\n", i,d_l0[i], i,d_b1[i],i, d_b0[i],i, d_bli[i]);
    }
}