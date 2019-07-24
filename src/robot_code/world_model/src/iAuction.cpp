#include <world_model/iAuction.h>
#include <limits>

void
iAuction::InitAlgo(){

    Deltas.resize(col_size, 0);
    assignment.resize(row_size, 0);
    allocation.resize(col_size, 0); // can initiate with -1, to distingush un-processed allocation, if later necessary

    record_allocations.resize(row_size, 0);
    pred.resize(row_size);
    pred_j.resize(col_size, -1);

    //initial assignment (possibly infeasible)
    for(uint i=0; i<row_size; i++){
        double max_d = -std::numeric_limits<double>::infinity();
        int max_j = -1;
        for(uint j=0; j<col_size; j++){
            if(orig_matrix[i][j] - Deltas[j] > max_d){
                max_d = orig_matrix[i][j] - Deltas[j];
                max_j = j;
            }
        }//j
        assert(max_j != -1);
        assignment[i] = max_j;
    }//i

    UpdateRecordAllocations();
    for(int i=0; i<row_size; i++){
        // update set of allocated cols
        allocated_cols.insert(assignment[i]);
        if(record_allocations[assignment[i]] == 1)
            // update initial allocation with info from assignment
            allocation[assignment[i]] = i;
    }

}


void
iAuction::UpdateRecordAllocations(void){

    // re scan num of assigned rows in each column
    record_allocations.clear();
    record_allocations.resize(col_size, 0);
    allocation_rows.clear();
    allocation_rows.resize(col_size);

    for(uint i=0; i<row_size; i++){
        record_allocations[ assignment[i] ] += 1;
        allocation_rows[assignment[i]].insert(i);
    }

}


void
iAuction::InitStage(){

    LU.clear();
    SV.clear();

    pred.clear();
    pred.resize(row_size);
    pred_j.clear();
    pred_j.resize(col_size, -1);
    
    heaps.clear();
    heaps.resize(row_size);

}


void
iAuction::Preprocess(void){

    for(uint i=0; i<row_size; i++)
        for(uint j=0; j<col_size; j++){
            if(j!=assignment[i]){
                pair<uint, double> p(j, orig_matrix[i][j]-Deltas[j]);
                heaps[i].push(p);
            }
        }
}


void
iAuction::MakeHeap(uint _row_id){

    for(uint j=0; j<col_size; j++){
        if(j!=assignment[_row_id]){
            pair<uint, double> p(j, orig_matrix[_row_id][j]-Deltas[j]);
            heaps[_row_id].push(p);
        }
    }

}



uint
iAuction::UpdateLabeledRows(uint _col_id){

    uint cnt=0;
    for(uint i=0; i<row_size; i++)
        if(assignment[i]==_col_id){
            LU.insert(i);
            MakeHeap(i);
            cnt++;
        }

    return cnt;
}



uint
iAuction::MainStage(uint s){

    InitStage();

    int sink = -1; //here sink is a column
    int i_bar= -1;
    int j_bar= -1;

    SV.insert(s);
    UpdateLabeledRows(s);
    while(sink == -1){
        double delta = std::numeric_limits<double>::infinity();
        for(set<uint>::iterator itr=LU.begin(); itr!=LU.end(); itr++){
            uint cur_j = heaps[*itr].top().first;
            while(SV.find(cur_j)!=SV.end()){
                heaps[*itr].pop();
                cur_j = heaps[*itr].top().first;
            }
            if(SV.find(cur_j) == SV.end()){
                if((orig_matrix[*itr][GetAssignedCol(*itr)]-Deltas[GetAssignedCol(*itr)]) - (orig_matrix[*itr][cur_j] - Deltas[cur_j])
                        < delta){
                    delta = (orig_matrix[*itr][GetAssignedCol(*itr)]-Deltas[GetAssignedCol(*itr)]) - (orig_matrix[*itr][cur_j] - Deltas[cur_j]);
                    j_bar = cur_j;
                    i_bar = *itr;
                }
            }//if SV
        }//for

        assert(j_bar != -1 && i_bar != -1);
        //pred[i_bar] = j_bar;
        pred[i_bar].push_back(j_bar);
        assert(pred_j[j_bar] == -1);
        pred_j[j_bar] = i_bar;
        for(set<uint>::iterator itr=SV.begin(); itr!=SV.end(); itr++)
            Deltas[*itr] += delta;
        price_cnt ++;

        if(!GetNumAsgnRows(j_bar)){
            sink = j_bar;
        }
        else{
            //update LU SV
            SV.insert(j_bar);
            UpdateLabeledRows(j_bar);
        }
#if 0
        //display data
        _cout("\tset SV: ");
        DisplaySet(SV);
        _cout("\tset LU: ");
        DisplaySet(LU);
        //_cout("\tpred: ");
        //DisplayVec<int>(pred);
        _cout("\tdalta="<<delta<<"; j*="<<j_bar<<endl);
        _cout("\tDeltas: ");
        DisplayVec<double>(Deltas);
        _cout(endl);
#endif
    }//while sink

    assert(sink!=-1);

    //update the assignment and allocation
    allocated_cols.insert(sink);
    int j_asgn=-1;
    uint cnt = 0;
    uint true_pred = sink;

    while(j_asgn!=(int)s){
        if(cnt++ > col_size)
            throw EXCEPTION_BROKEN;
        j_asgn=assignment[i_bar];
        assignment[i_bar] = true_pred;
#if 0
        _cout("\ti_bar="<<i_bar<<" j_asgn="<<j_asgn<<" pred="<<true_pred<<endl);
#endif

        if(pred_j[j_asgn] != -1){
            i_bar=pred_j[j_asgn];
            for(uint j=0; j<pred[i_bar].size(); j++)
                if((int)pred[i_bar][j] == j_asgn){
                    true_pred = pred[i_bar][j];
                    break;
                }
        }
        //assert(i<row_size || j_asgn==(int)s);
    }//while

    //update num_allocation, by re-scanning num of assigned rows in each column
    UpdateRecordAllocations();

#if 0
    _cout("Stats of this stage:"<<endl);
    _cout("\tstart col = "<<s<<"; sink col = "<<sink<<endl);
    _cout("\tassignment: ");
    DisplayVec<uint>(assignment);
    _cout("\tallocation: ");
    DisplayVec<uint>(allocation);
    _cout("\trecord_allocations: ");
    DisplayVec<uint>(record_allocations);
    _cout("transformed matrix with current assignment and Deltas: "<<endl);
    DisplayMatrix(orig_matrix, assignment, Deltas);
    _cout("-----------------------------------------------------"<<endl);
#endif

    return (uint)sink;
}


void
iAuction::MainAlgo(){

    InitAlgo();

    lu_seq.clear();
    size_sum=0;

    while(allocated_cols.size()<row_size){

        int next = -1;
        for(uint j=0; j<col_size; j++)
            if(record_allocations[j]>1){
                next = j;
                break;
            }
        assert(next!=-1);

        //   _cout("Selected column: "<<next<<endl);
        //Preprocess();
        MainStage(next);

    }//while

    //DoubleCheck();
#if 0 
    cout<<"Final solution:"<<endl;
    DisplayMatrix(orig_matrix, assignment, true);
    cout<<"Assigned row-col pairs: "<<endl;
    DisplayAssignment();
    cout<<"Maximization result: "<<ComputeCostSum(orig_matrix, assignment)<<endl;
#endif
}



double
iAuction::ComputeCostSum(const mat& _m, const vector<uint>& _as) const{

    double sum = 0;
    for(uint i=0; i<_m.size(); i++){
        for(uint j=0; j<_m[0].size(); j++)
            if(_as[i] == j)
                sum += _m[i][j];
    }

    return sum;

}

void
iAuction::DoubleCheck(void){

    for(uint j=0; j<col_size; j++)
        if(record_allocations[j]!=1)
            throw EXCEPTION_WRONG;

}


// some display functions
void
iAuction::DisplayMatrix(const mat& _m) const{

    if(_m[0].size() > DISPLAY_WIDTH){
        _cout("Matrix is big, not displaying."<<endl);
        return;
    }

    for(uint i=0; i<_m.size(); i++){
        for(uint j=0; j<_m[0].size(); j++)
            _cout(" "<<_m[i][j]<<"\t");
        _cout(endl);
    }

}

void
iAuction::DisplayMatrix(const mat& _m, const vector<uint>& _as, bool must_display) const{

    if(must_display == false){
        if(_m[0].size() > DISPLAY_WIDTH){
            _cout("Matrix is big, not displaying."<<endl);
            return;
        }

        for(uint i=0; i<_m.size(); i++){
            for(uint j=0; j<_m[0].size(); j++){
                if(_as[i] == j)
                    _cout(" "<<_m[i][j]<<"*\t");
                else
                    _cout(" "<<_m[i][j]<<"\t");
            }
            _cout(endl);
        }
    }else{
        for(uint i=0; i<_m.size(); i++){
            for(uint j=0; j<_m[0].size(); j++){
                if(_as[i] == j)
                    cout<<" "<<_m[i][j]<<"*\t";
                else
                    cout<<" "<<_m[i][j]<<"\t";
            }
            cout<<endl;
        }

    }//end for

}

void
iAuction::DisplayMatrix(const mat& _m, const vector<uint>& _as, const vector<double>& _deltas) const{

    if(_m[0].size() > DISPLAY_WIDTH){
        _cout("Matrix is big, not displaying."<<endl);
        return;
    }

    for(uint i=0; i<_m.size(); i++){
        for(uint j=0; j<_m[0].size(); j++){
            if(_as[i] == j)
                _cout(" "<<_m[i][j]-_deltas[j]<<"*\t");
            else
                _cout(" "<<_m[i][j]-_deltas[j]<<"\t");
        }
        _cout(endl);
    }

}



void
iAuction::DisplayAssignment(void) const{

    for(uint i=0; i<assignment.size(); i++)
        cout<<"("<<i<<","<<assignment[i]<<") ";
    cout<<endl;

}


// void
// iAuction::DisplaySet(const set<uint>& _s) const{
// 
//   for(set<uint>::iterator itr=_s.begin(); itr!=_s.end(); itr++)
//     _cout(*itr<<" "); 
//   _cout(endl);
// 
// }


//templated function not defined in header, so can only used in this .cpp
template<typename T>
void
iAuction::DisplayVec(const vector<T>& _vec){

    for(typename vector<T>::const_iterator itr=_vec.begin(); itr!=_vec.end(); itr++)
        _cout(*itr<<" ");
    _cout(endl);

}



