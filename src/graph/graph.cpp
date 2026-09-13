#include "graph.hpp"

namespace gd2026 {

void Graph::initialize(int32_t num_nodes, int32_t num_edges) {
    assert(num_nodes >= 0);
    assert(num_edges >= 0);

    m_original_num_nodes = num_nodes;
    m_original_num_edges = num_edges;

    m_x.assign(static_cast<size_t>(num_nodes), 0);
    m_y.assign(static_cast<size_t>(num_nodes), 0);
    m_parent.assign(static_cast<size_t>(num_nodes), config::INVALID_ID);

    m_edges.assign(static_cast<size_t>(num_edges), Edge{config::INVALID_ID, config::INVALID_ID, 0, 0});
    m_head.assign(static_cast<size_t>(num_nodes), config::INVALID_ID);
    m_next.assign(static_cast<size_t>(num_edges), config::INVALID_ID);
    m_incident_head.assign(static_cast<size_t>(num_nodes + 1), 0);
    m_incident_edges.clear();
    m_incident_write.clear();
}

void Graph::build_forward_star() {
    m_head.assign(static_cast<size_t>(m_original_num_nodes), config::INVALID_ID);
    m_next.assign(m_edges.size(), config::INVALID_ID);
    m_incident_head.assign(static_cast<size_t>(m_original_num_nodes + 1), 0);

    for (int32_t e = 0; e < m_original_num_edges; ++e) {
        const Edge& edge = m_edges[static_cast<size_t>(e)];
        assert(edge.u >= 0 && edge.u < m_original_num_nodes);
        assert(edge.v >= 0 && edge.v < m_original_num_nodes);
        ++m_incident_head[static_cast<size_t>(edge.u + 1)];
        ++m_incident_head[static_cast<size_t>(edge.v + 1)];
    }
    for (int32_t u = 1; u <= m_original_num_nodes; ++u) {
        m_incident_head[static_cast<size_t>(u)] += m_incident_head[static_cast<size_t>(u - 1)];
    }
    m_incident_edges.assign(static_cast<size_t>(m_incident_head.back()), config::INVALID_ID);
    m_incident_write.assign(m_incident_head.begin(), m_incident_head.end());
    for (int32_t e = 0; e < m_original_num_edges; ++e) {
        const Edge& edge = m_edges[static_cast<size_t>(e)];
        m_incident_edges[static_cast<size_t>(m_incident_write[static_cast<size_t>(edge.u)]++)] = e;
        m_incident_edges[static_cast<size_t>(m_incident_write[static_cast<size_t>(edge.v)]++)] = e;
    }

    const int32_t edge_limit = (m_original_num_edges < static_cast<int32_t>(m_edges.size()))
        ? m_original_num_edges
        : static_cast<int32_t>(m_edges.size());

    for (int32_t e = 0; e < edge_limit; ++e) {
        const int32_t u = m_edges[static_cast<size_t>(e)].u;
        assert(u >= 0 && u < m_original_num_nodes);

        m_next[static_cast<size_t>(e)] = m_head[static_cast<size_t>(u)];
        m_head[static_cast<size_t>(u)] = e;
    }
}

void Graph::pad_edges_for_simd() {
    const size_t remainder = m_edges.size() % config::SIMD_WIDTH;
    if (remainder == 0) {
        return;
    }

    const size_t padding = config::SIMD_WIDTH - remainder;
    m_edges.insert(
        m_edges.end(),
        padding,
        Edge{config::INVALID_ID, config::INVALID_ID, 0, 0}
    );
    m_next.resize(m_edges.size(), config::INVALID_ID);
}

} // namespace gd2026