//
// Created by Khurram Javed on 2023-03-18.
//

#include "../../include/nn/architure_initializer.h"
#include <iostream>

Graph *ArchitectureInitializer::initialize_sprase_networks(
    Graph *my_graph, int total_parameters, int parameters_per_feature,
    std::string vertex_type, float step_size) {

  int total_features = int(total_parameters / parameters_per_feature);
  for (int i = 0; i < total_features; i++) {
    my_graph->list_of_vertices.push_back(
        VertexFactory::get_vertex(vertex_type));
    auto sampler = std::uniform_int_distribution<int>(
        0, my_graph->list_of_vertices.size() - 2);
    for (int j = 0; j < parameters_per_feature; j++) {
      my_graph->add_edge(0, sampler(my_graph->mt), my_graph->list_of_vertices.size()-1,
                         step_size);
    }
  }

  Vertex *prediction_vertex = VertexFactory::get_vertex("linear");
  my_graph->list_of_vertices.push_back(prediction_vertex);

  my_graph->list_of_vertices[my_graph->list_of_vertices.size() - 1]->is_output =
      true;

  for (int i = 0; i < my_graph->list_of_vertices.size() - 1; i++) {
    my_graph->add_edge(0, i, my_graph->list_of_vertices.size()-1, step_size);
  }
  return my_graph;
}