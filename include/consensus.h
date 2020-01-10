//
// Created by miguel on 03/12/19.
//

#ifndef SCDTR_1_CONSENSUS_H
#define SCDTR_1_CONSENSUS_H


class c_node {
    public:
        int index{};
        int lenght;
        float* d{};
        float* d_av{};
        float* y{};
        float* k{};
        float n{};
        float m{};
        float* c{};
        float o{};
        float L{};
        float rho{};
        float* next_it;
        float cost_it;
        
        c_node(int lenght, int id_in_array, float* KARR, float own_c,float own_o, float own_L, float rho);
        float evaluate_cost(float d_aux[]);
        float dotProduct(const float vectA[], const float vectB[]);
        void VectDifference(float vectA[],float vectB[], float diff[]);
        bool check_feasibility(float d_aux[]);
        float sum_of_squares(float *vect);
        void consensus_iterate();
        void update_average(float values[], int index);
        void update_lagrange();
};


#endif //SCDTR_1_CONSENSUS_H
