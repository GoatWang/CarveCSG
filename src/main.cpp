#include <list>
#include <vector>
#include <iostream>

#include <carve/csg.hpp>
#include <carve/input.hpp>
#include <carve/convex_hull.hpp>

#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>


static carve::mesh::MeshSet<3> *makeCube(const carve::math::Matrix &transform) {
  carve::input::PolyhedronData data;

  data.addVertex(transform * carve::geom::VECTOR(+1.0, +1.0, +1.0));
  data.addVertex(transform * carve::geom::VECTOR(-1.0, +1.0, +1.0));
  data.addVertex(transform * carve::geom::VECTOR(-1.0, -1.0, +1.0));
  data.addVertex(transform * carve::geom::VECTOR(+1.0, -1.0, +1.0));
  data.addVertex(transform * carve::geom::VECTOR(+1.0, +1.0, -1.0));
  data.addVertex(transform * carve::geom::VECTOR(-1.0, +1.0, -1.0));
  data.addVertex(transform * carve::geom::VECTOR(-1.0, -1.0, -1.0));
  data.addVertex(transform * carve::geom::VECTOR(+1.0, -1.0, -1.0));

  data.addFace(0, 1, 2, 3);
  data.addFace(7, 6, 5, 4);
  data.addFace(0, 4, 5, 1);
  data.addFace(1, 5, 6, 2);
  data.addFace(2, 6, 7, 3);
  data.addFace(3, 7, 4, 0);

  return new carve::mesh::MeshSet<3>(data.points, data.getFaceCount(), data.faceIndices);
}

void testCSG(carve::mesh::MeshSet<3>* a, carve::mesh::MeshSet<3>* b, carve::csg::CSG::OP& op, carve::mesh::MeshSet<3>*& finalResult) {
    if (!a || !b) return;

    try {
        carve::mesh::MeshSet<3> *result = NULL;
        // result = carve::csg::CSG().compute(a, b, op);
        result = carve::csg::CSG().compute(a, b, op, NULL, carve::csg::CSG::CLASSIFY_NORMAL);

        std::cerr << "result "
                  << result << " has " << result->meshes.size()
                  << " manifolds (" << std::count_if(result->meshes.begin(),
                                                     result->meshes.end(),
                                                     carve::mesh::Mesh<3>::IsClosed()) << " closed)" << std::endl; 
    
        std::cerr << "closed:    ";
        for (size_t i = 0; i < result->meshes.size(); ++i) {
          std::cerr << (result->meshes[i]->isClosed() ? '+' : '-');
        }
        std::cerr << std::endl;
    
        std::cerr << "negative:  ";
        for (size_t i = 0; i < result->meshes.size(); ++i) {
          std::cerr << (result->meshes[i]->isNegative() ? '+' : '-');
        }
        std::cerr << std::endl;

        // auto mesh = result->meshes[0];
        // std::cout << "n_faces (result): " << mesh->faces.size() << std::endl;
        // std::vector<carve::mesh::Vertex<3> *> v;
        // for (size_t i = 0, l = mesh->faces.size(); i != l; ++i) {
        //     pcl::Vertices polygon_pcl;
        //     carve::mesh::Face<3> *f = mesh->faces[i];
        //     f->getVertices(v);
        //     std::cout << "n_vertices(face " << i << ") : " << v.size() << std::endl;
        // }

        // Place the result of this CSG into our final result, and get rid of our last one
        std::swap(result, finalResult);
        delete result;
      } 
      catch (carve::exception e) {
        std::cerr << "FAIL- " << e.str();
        if (finalResult) delete finalResult;
        finalResult = NULL;
      }
}

void get_vertices_map_and_cloud(carve::mesh::Mesh<3>* mesh, std::map<std::uintptr_t, size_t>& v_map, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl){
    size_t vid_order = 0;
    std::vector<carve::mesh::Vertex<3> *> v;
    for (size_t i = 0, l = mesh->faces.size(); i != l; ++i) {
        carve::mesh::Face<3> *f = mesh->faces[i];
        f->getVertices(v);
        for (size_t vid=0; vid<v.size(); vid++){
            std::uintptr_t v_ptr = reinterpret_cast<std::uintptr_t>(v[vid]);
            if (v_map.count(v_ptr) == 0){
                float x = static_cast<float>(v[vid]->v.x);
                float y = static_cast<float>(v[vid]->v.y);
                float z = static_cast<float>(v[vid]->v.z);
                cloud_pcl->points.push_back(pcl::PointXYZ(x, y, z));
                v_map[v_ptr] = vid_order;
                vid_order += 1;
            }
        }
    }
}

void _get_mesh_pcl(
    carve::mesh::Mesh<3>* mesh,
    std::map<std::uintptr_t, size_t>& v_map,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_pcl,
    pcl::PolygonMesh& mesh_pcl
    ){
    pcl::toPCLPointCloud2(*cloud_pcl, mesh_pcl.cloud);
    std::vector<carve::mesh::Vertex<3> *> v;
    for (size_t i = 0, l = mesh->faces.size(); i != l; ++i) {
        carve::mesh::Face<3> *f = mesh->faces[i];
        f->getVertices(v);
        pcl::Vertices polygon;
        for (size_t vid=0; vid<v.size(); vid++){
            std::uintptr_t v_ptr = reinterpret_cast<std::uintptr_t>(v[vid]);
            polygon.vertices.push_back(v_map[v_ptr]);
        }
        mesh_pcl.polygons.push_back(polygon);
    }
}

void convert_mesh_pcl(carve::mesh::Mesh<3>* mesh, pcl::PolygonMesh& mesh_pcl){
    std::map<std::uintptr_t, size_t> v_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    get_vertices_map_and_cloud(mesh, v_map, cloud_pcl);
    _get_mesh_pcl(mesh, v_map, cloud_pcl, mesh_pcl);
}

void show_carve_mesh(pcl::visualization::PCLVisualizer& viewer, std::string name, carve::mesh::Mesh<3>* mesh){
    pcl::PolygonMesh mesh_pcl;
    convert_mesh_pcl(mesh, mesh_pcl);
    viewer.addPolygonMesh(mesh_pcl, name);
}

void _show_line_pcl(
    pcl::visualization::PCLVisualizer& viewer, 
    const pcl::PointXYZ& p0, 
    const pcl::PointXYZ& p1,
    std::string line_id,
    int r, int g, int b, float line_width 
    ){
    viewer.addLine(p0, p1, line_id);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
                                        static_cast<float>(r)/255, 
                                        static_cast<float>(g)/255, 
                                        static_cast<float>(b)/255, line_id); 
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, line_id); 
}

void show_carve_mesh_edges(pcl::visualization::PCLVisualizer& viewer, std::string name, carve::mesh::Mesh<3>* mesh, int r, int g, int b, int line_width){
    // carve::geom3d::AABB aabb = mesh->getAABB();
    // carve::geom3d::Vector min = aabb.min();
    // carve::geom3d::Vector max = aabb.max();
    for (size_t i = 0, l = mesh->closed_edges.size(); i != l; ++i) {
        carve::mesh::Edge<3> *edge = mesh->closed_edges[i];
        carve::geom3d::Vector p0 = edge->v1()->v;
        carve::geom3d::Vector p1 = edge->v2()->v;
        pcl::PointXYZ p0_pcl(p0.x, p0.y, p0.z);
        pcl::PointXYZ p1_pcl(p1.x, p1.y, p1.z);
        _show_line_pcl(viewer, p0_pcl, p1_pcl, name + std::to_string(i), r, g, b, line_width);
    }    
}

int main() {
    int test = 1;  // Set the test case directly here

    // std::list<carve::mesh::MeshSet<3>*> meshes;
    // std::list<carve::csg::CSG::OP> ops;
    // getInputsFromTest(test, meshes, ops);

    carve::mesh::MeshSet<3> *a = makeCube(carve::math::Matrix::SCALE(2.0, 2.0, 2.0));
    carve::mesh::MeshSet<3> *b = makeCube(carve::math::Matrix::SCALE(2.0, 2.0, 2.0) *
                                            carve::math::Matrix::ROT(1.0, 1.0, 1.0, 1.0) *
                                            carve::math::Matrix::TRANS(1.0, 1.0, 1.0));    

    carve::mesh::MeshSet<3> *meshset_dst = NULL;
    carve::csg::CSG::OP op_intersection = carve::csg::CSG::INTERSECTION;
    testCSG(a, b, op_intersection, meshset_dst);
    

    carve::mesh::MeshSet<3> *meshset_union_dst = NULL;
    carve::csg::CSG::OP op_union = carve::csg::CSG::UNION;
    testCSG(a, b, op_union, meshset_union_dst);

    pcl::visualization::PCLVisualizer viewer("CSG Result Viewer");
    // show_carve_mesh_edges(viewer, "a", a->meshes[0], 255, 0, 0, 10);
    // show_carve_mesh_edges(viewer, "b", b->meshes[0], 0, 0, 255, 10);
    show_carve_mesh_edges(viewer, "meshset_union_dst", meshset_union_dst->meshes[0], 0, 0, 255, 10);
    show_carve_mesh(viewer, "mesh_dst", meshset_dst->meshes[0]);
    viewer.spin();

    std::cout << "operation success!" << std::endl;


    return 0;
}
