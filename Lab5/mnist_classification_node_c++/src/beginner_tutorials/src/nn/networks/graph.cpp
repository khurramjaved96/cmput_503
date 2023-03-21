//
// Created by Khurram Javed on 2022-08-30.
//

#include "../../../include/nn/networks/graph.h"
#include "../../../include/nn/networks/vertex.h"
#include <iostream>
#include <random>
#include <string>
#include <vector>

// Graph implementation

void Graph::add_edge(float weight, int from, int to, float step_size) {
  Edge my_edge = Edge(weight, from, to, step_size);
  list_of_vertices[to]->incoming_edges.push_back(my_edge);
  list_of_vertices[from]->sum_of_outgoing_weights += std::abs(weight);
}

void Graph::prune_weight() {
  int address_min = -1;
  float value_min = 1000000;
  for (int i = list_of_vertices.size() - 1; i >= 0; i--) {
    int counter = 0;
    if (list_of_vertices[i]->utility_trace < value_min) {
      address_min = i;
      value_min = list_of_vertices[i]->utility_trace;
    }
  }
  int size = list_of_vertices[address_min]->incoming_edges.size();
  list_of_vertices[address_min]->incoming_edges.clear();
}

void Graph::print_graph() {
  int counter = 0;
  std::cout << "digraph g {\n";
  for (auto v : this->list_of_vertices) {
    std::cout << v->id << " [label=\"" << std::to_string(counter) << "\"];"
              << std::endl;
    for (auto incoming : v->incoming_edges) {
      std::cout << incoming.from << " -> " << incoming.to << "[label=\""
                << std::to_string(incoming.weight) << "\"];" << std::endl;
    }
    counter++;
  }
  std::cout << "}\n";
}

std::string Graph::serialize_graph() {
  std::string empty_string;
  int counter = 0;
  for (auto v : this->list_of_vertices) {
//    empty_string += std::to_string(v->id) + " [label=\"" +
//                    std::to_string(counter) + "\"];\n";
    for (auto incoming : v->incoming_edges) {
      empty_string += std::to_string(incoming.from)  + ", " + std::to_string(incoming.to) + ", " + std::to_string(incoming.weight);
    }
    counter++;
  }
//  empty_string += "}\n";
//  std::cout << empty_string << std::endl;
  return empty_string;
}

float Graph::get_prediction() { return this->prediction; }

void Graph::print_utility() {
  for (int i = 0; i < this->list_of_vertices.size(); i++) {
    std::cout << "Vertex " << i << std::endl;
    //    std::cout << "Vertex value = " << this->list_of_vertices[i]->forward()
    //    << std::endl;
    for (auto incoming : this->list_of_vertices[i]->incoming_edges) {
      std::cout << "Incoming edge util = " << incoming.utility << std::endl;
      //      std::cout << "Incoming edge grad = " << incoming.gradient <<
      //      std::endl;
    }
  }
}

Graph::Graph(int input_vertices, int seed) : mt(seed) {

  this->input_vertices = input_vertices;
  Vertex *mem = new Vertex[input_vertices];
  for (int i = 0; i < input_vertices; i++) {
    this->list_of_vertices.push_back(&mem[i]);
  }
}

void Graph::set_input_values(std::vector<float> inp) {
  for (int i = 0; i < input_vertices; i++) {
    this->list_of_vertices[i]->value = inp[i];
  }
}

std::vector<int> Graph::get_distribution_of_values() {
  std::vector<int> distributon_of_values;
  for (int i = 0; i < 11; i++) {
    distributon_of_values.push_back(0);
  }
  for (int i = input_vertices; i < list_of_vertices.size(); i++) {
    float temp_val = this->list_of_vertices[i]->forward();
    //    std::cout << temp_val << std::endl;
    if (temp_val > this->list_of_vertices[i]->max_value)
      temp_val = this->list_of_vertices[i]->max_value;
    if (temp_val < this->list_of_vertices[i]->min_value)
      temp_val = this->list_of_vertices[i]->min_value;

    float range = this->list_of_vertices[i]->max_value -
                  this->list_of_vertices[i]->min_value;
    float bin_size = range / 10;
    int bin_number = int(temp_val / bin_size);
    distributon_of_values[bin_number]++;
  }
  return distributon_of_values;
}

float Graph::update_values() {
  for (int i = input_vertices; i < list_of_vertices.size(); i++) {
    list_of_vertices[i]->value = 0;
    for (auto &e : list_of_vertices[i]->incoming_edges) {
      e.gradient = 0;
      list_of_vertices[i]->value +=
          list_of_vertices[e.from]->forward() * e.weight;
    }
  }
  this->prediction = list_of_vertices[list_of_vertices.size() - 1]->forward();
  return this->prediction;
}

void Graph::estimate_gradient(float error) {
  for (int i = 0; i < list_of_vertices.size(); i++) {
    this->list_of_vertices[i]->d_out_d_vertex = 0;
    this->list_of_vertices[i]->d_out_d_vertex_before_non_linearity = 0;
  }

  //  Back-prop implementation
  this->list_of_vertices[list_of_vertices.size() - 1]->d_out_d_vertex = 1;
  this->list_of_vertices[list_of_vertices.size() - 1]
      ->d_out_d_vertex_before_non_linearity = 1;
  for (int i = list_of_vertices.size() - 1; i >= 0; i--) {
    this->list_of_vertices[i]->d_out_d_vertex_before_non_linearity =
        this->list_of_vertices[i]->d_out_d_vertex;
    this->list_of_vertices[i]->d_out_d_vertex =
        this->list_of_vertices[i]->backward(this->list_of_vertices[i]->value) *
        this->list_of_vertices[i]->d_out_d_vertex;
    for (auto &e : this->list_of_vertices[i]->incoming_edges) {
      e.gradient = this->list_of_vertices[e.from]->forward() *
                   this->list_of_vertices[e.to]->d_out_d_vertex * error;
      e.temp_gradient = this->list_of_vertices[e.from]->forward() *
                        this->list_of_vertices[e.to]->d_out_d_vertex;
      this->list_of_vertices[e.from]->d_out_d_vertex +=
          this->list_of_vertices[e.to]->d_out_d_vertex * e.weight;
    }
  }
}

float Graph::get_average_gradient(float error) {
  float sum_of_gradients = 0;
  int counter = 0;
  for (int i = list_of_vertices.size() - 1; i >= 0; i--) {
    if (this->list_of_vertices[i]->type == "sigmoid" ||
        this->list_of_vertices[i]->type == "tanh" ||
        this->list_of_vertices[i]->type == "leakyrelu" ||
        this->list_of_vertices[i]->type == "relu") {
      for (auto &e : this->list_of_vertices[i]->incoming_edges) {
        sum_of_gradients += std::abs(e.temp_gradient);
        counter += 1;
      }
    }
  }
  return sum_of_gradients / counter;
}

Graph::Graph() {}
int Graph::GetInputVertices() const { return input_vertices; }
void Graph::SetInputVertices(int input_vertices) {
  Graph::input_vertices = input_vertices;
}

GraphLinearAssumptionUtility::GraphLinearAssumptionUtility(
    int input_vertices, int seed, float utility_decay_rate)
    : Graph(input_vertices, seed) {
  this->utility_decay_rate = utility_decay_rate;
}

GradientUtility::GradientUtility(int input_vertices, int seed,
                                 float utility_decay_rate)
    : Graph(input_vertices, seed) {
  this->utility_decay_rate = utility_decay_rate;
}

GradientLocalUtility::GradientLocalUtility(int input_vertices, int seed,
                                           float utility_decay_rate)
    : Graph(input_vertices, seed) {
  this->utility_decay_rate = utility_decay_rate;
}

UtilityPropagation::UtilityPropagation(int input_vertices, int seed,
                                       float utility_decay_rate)
    : Graph(input_vertices, seed) {
  this->utility_decay_rate = utility_decay_rate;
}

ActivationTrace::ActivationTrace(int input_vertices, int seed,
                                 float utility_decay_rate)
    : Graph(input_vertices, seed) {
  this->utility_decay_rate = utility_decay_rate;
}

WeightUtility::WeightUtility(int input_vertices, int seed)
    : Graph(input_vertices, seed) {}

RandomUtility::RandomUtility(int input_vertices, int seed)
    : Graph(input_vertices, seed) {
  std::uniform_real_distribution<float> rand_gen(0, 4);
  for (int i = 0; i < this->list_of_vertices.size(); i++) {
    for (auto &e : this->list_of_vertices[i]->incoming_edges) {
      e.utility = rand_gen(this->mt);
    }
  }
}

GraphLocalUtility::GraphLocalUtility(int input_vertices, int seed,
                                     float utility_decay_rate)
    : Graph(input_vertices, seed) {
  this->utility_decay_rate = utility_decay_rate;
}
