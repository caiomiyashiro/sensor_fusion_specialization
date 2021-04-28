#ifndef ring_buffer_hpp
#define ring_buffer_hpp
#include <iostream>
#include <vector>

using namespace std;

template<class T>
class RingBuffer{
    public:
        vector<T> buffer;
        int max_size, head, tail, n_elements;

        RingBuffer(int max_size);
        void push(T elem);
        void clean();
        int get_size();
        T pop();
        T* get_newest_elem_reference();
        T* get_oldest_elem_reference();
        
};

//////////////////////////////////////////

template<typename T>
RingBuffer<T>::RingBuffer(int max_size){
    this->max_size = max_size;
    this->buffer.resize(max_size);
    this->head = 0;
    this->tail = 0;
    this->n_elements = 0;

}

template<typename T>
void RingBuffer<T>::clean(){
    this->head = 0;
    this->tail = 0;
    this->n_elements = 0;
}

template<typename T>
int RingBuffer<T>::get_size(){
    return this->n_elements;
}

template<typename T>
void RingBuffer<T>::push(T elem){
    if(this->n_elements == 0){
        this->buffer[this->head] = elem;
        this->n_elements++;
    }
    else if(this->n_elements < this->max_size){
        this->head = ((this->head)+1)%this->max_size;
        this->buffer[this->head] = elem;
        this->n_elements++;
    }
    else if(this->n_elements == this->max_size){
        this->head = ((this->head)+1)%this->max_size;
        this->buffer[this->head] = elem;
        this->tail = ((this->tail)+1)%this->max_size;
    }
    // cout << "this->head " << this->head << "this->tail " << this->tail << endl; 
}

template<typename T>
T RingBuffer<T>::pop(){
    if(this->n_elements == 0){
        return NULL;
    }
    else{
        T elem = this->buffer[this->tail];
        this->tail = ((this->tail)+1)%this->max_size;
        this->n_elements--;
        // cout << "this->head " << this->head << "this->tail " << this->tail << endl; 
        return elem;
    }
}

template<typename T>
T* RingBuffer<T>::get_newest_elem_reference(){
    return &this->buffer[this->head];
}

template<typename T>
T* RingBuffer<T>::get_oldest_elem_reference(){
    return &this->buffer[this->tail];
}

//////////////////////////////////////////

template<typename T>
std::ostream& operator<<(ostream& os, const RingBuffer<T> ring_buffer)
{
    if(ring_buffer.n_elements == 0){
        os << "empty ring buffer";
    }else{
        stringstream ss;
        int support_counter = 0;
        int support_ix = ring_buffer.tail;
        ss << "Sorted Elements -";
        // ss << ring_buffer.buffer[0] << " " << ring_buffer.buffer[1] << endl;
        while(support_counter < ring_buffer.n_elements){
            ss << " " << ring_buffer.buffer[support_ix];
            support_ix = (support_ix+1)%ring_buffer.max_size;
            support_counter++;
        }
        os << ss.str();
    }
    return os;
}

#endif