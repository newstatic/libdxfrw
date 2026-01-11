/**
 * Python bindings for libdxfrw using pybind11
 *
 * This module provides Python access to DWG/DXF file reading capabilities.
 * Supports block expansion for complete drawing content.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <cmath>
#include <map>
#include <iostream>
#include <cstdio>

#include "../src/libdxfrw.h"
#include "../src/libdwgr.h"
#include "../src/drw_entities.h"
#include "../src/drw_objects.h"
#include "../src/drw_interface.h"
#include "../src/intern/drw_textcodec.h"

namespace py = pybind11;

// Global debug flag (default: disabled for production)
static bool g_debugEnabled = false;

void setDebugEnabled(bool enabled) {
    g_debugEnabled = enabled;
}

bool isDebugEnabled() {
    return g_debugEnabled;
}

// Python-friendly data structures
struct PyCoord {
    double x, y, z;
    PyCoord() : x(0), y(0), z(0) {}
    PyCoord(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
    PyCoord(const DRW_Coord& c) : x(c.x), y(c.y), z(c.z) {}

    // Transform by position, scale, rotation
    PyCoord transform(const PyCoord& pos, double xScale, double yScale, double angle) const {
        // Apply scale
        double sx = x * xScale;
        double sy = y * yScale;

        // Apply rotation
        double cosA = cos(angle);
        double sinA = sin(angle);
        double rx = sx * cosA - sy * sinA;
        double ry = sx * sinA + sy * cosA;

        // Apply translation
        return PyCoord(rx + pos.x, ry + pos.y, z + pos.z);
    }
};

struct PyLayer {
    std::string name;
    int color;
    bool is_on;
    std::string lineType;
};

struct PyText {
    std::string content;
    PyCoord position;
    double height;
    double angle;
    std::string layer;
    std::string style;
};

struct PyMText {
    std::string content;
    PyCoord position;
    double height;
    double angle;
    std::string layer;
    std::string style;
    double interlin;
};

struct PyLine {
    PyCoord start;
    PyCoord end;
    std::string layer;
};

struct PyCircle {
    PyCoord center;
    double radius;
    std::string layer;
};

struct PyArc {
    PyCoord center;
    double radius;
    double startAngle;
    double endAngle;
    std::string layer;
};

struct PyPolylineVertex {
    double x, y, z;
    double bulge;
};

struct PyLWPolyline {
    std::vector<PyPolylineVertex> vertices;
    bool closed;
    std::string layer;
    double elevation;
};

struct PyInsert {
    std::string blockName;
    PyCoord position;
    double xScale, yScale, zScale;
    double angle;
    std::string layer;
};

// Ellipse structure (defined before PyBlock since PyBlock contains ellipses)
struct PyEllipse {
    PyCoord center;          // Center point (from basePoint)
    PyCoord majorAxis;       // Major axis endpoint (from secPoint, relative to center)
    double ratio;            // Ratio of minor to major axis
    double startAngle;       // Start parameter (0 for full ellipse)
    double endAngle;         // End parameter (2*PI for full ellipse)
    std::string layer;
};

struct PyBlock {
    std::string name;
    PyCoord basePoint;
    int flags;
    // Block content - entities within the block
    std::vector<PyLine> lines;
    std::vector<PyCircle> circles;
    std::vector<PyArc> arcs;
    std::vector<PyLWPolyline> lwpolylines;
    std::vector<PyText> texts;
    std::vector<PyMText> mtexts;  // MTEXT entities (used for dimension text in *D blocks)
    std::vector<PyEllipse> ellipses;
    std::vector<PyInsert> inserts;  // Nested block references
};

// Dimension structure with complete geometry
struct PyDimension {
    int type;                // Dimension type (0=linear, 1=aligned, 2=angular, etc.)
    PyCoord defPoint;        // Definition point (dimension line location)
    PyCoord textPoint;       // Text middle point
    PyCoord def1;            // Definition point 1 (start of extension line 1)
    PyCoord def2;            // Definition point 2 (start of extension line 2)
    PyCoord circlePoint;     // For radial/diameter dimensions
    PyCoord arcPoint;        // Arc point for angular dimensions
    double angle;            // Rotation angle
    double length;           // Leader length (for radial)
    std::string text;        // Explicit dimension text
    std::string style;       // Dimension style name
    std::string layer;
    std::string blockName;   // Name of the *D block that contains the dimension geometry
};

// Hatch boundary edge (line, arc, or ellipse segment)
struct PyHatchEdge {
    int type;                // 1=line, 2=arc, 3=ellipse, 4=spline
    // For line (type=1)
    PyCoord start;
    PyCoord end;
    // For arc (type=2)
    PyCoord center;
    double radius;
    double startAngle;
    double endAngle;
    bool ccw;
    // For ellipse (type=3)
    PyCoord majorAxis;
    double ratio;
};

// Hatch boundary loop
struct PyHatchLoop {
    int type;                // Loop type (2=polyline, 0=edges)
    std::vector<PyHatchEdge> edges;
    // For polyline loops
    std::vector<PyPolylineVertex> vertices;
    bool closed;
};

// Hatch structure with boundary paths
struct PyHatch {
    std::string name;        // Pattern name
    int solid;               // 1=solid fill, 0=pattern
    std::string layer;
    double angle;            // Pattern angle
    double scale;            // Pattern scale
    std::vector<PyHatchLoop> loops;  // Boundary loops

    PyHatch() : solid(0), angle(0), scale(1.0) {}
};

// Main data container returned to Python
struct PyDxfData {
    std::string filename;
    std::string version;
    bool valid;
    std::string error;

    // Collections - Model Space entities
    std::vector<PyLayer> layers;
    std::vector<PyText> texts;
    std::vector<PyMText> mtexts;
    std::vector<PyLine> lines;
    std::vector<PyCircle> circles;
    std::vector<PyArc> arcs;
    std::vector<PyLWPolyline> lwpolylines;
    std::vector<PyEllipse> ellipses;
    std::vector<PyInsert> inserts;
    std::vector<PyBlock> blocks;
    std::vector<PyDimension> dimensions;
    std::vector<PyHatch> hatches;

    // Expanded entities (after block expansion)
    std::vector<PyLine> expandedLines;
    std::vector<PyCircle> expandedCircles;
    std::vector<PyArc> expandedArcs;
    std::vector<PyLWPolyline> expandedPolylines;
    std::vector<PyText> expandedTexts;
    std::vector<PyMText> expandedMTexts;  // Expanded MTEXT (from blocks including *D dimension blocks)
    std::vector<PyEllipse> expandedEllipses;

    // Statistics
    int totalEntities;
    int totalLayers;
    int totalBlocks;
};

// Implementation of DRW_Interface that collects data
class PyDxfCollector : public DRW_Interface {
public:
    PyDxfData data;
    std::string currentBlockName;
    bool inBlock;
    std::map<std::string, PyBlock*> blockMap;  // Map block name to block pointer

    PyDxfCollector() : inBlock(false) {
        data.valid = true;
        data.totalEntities = 0;
        data.totalLayers = 0;
        data.totalBlocks = 0;
    }

    // Header
    void addHeader(const DRW_Header* header) override {
        // Extract version info if needed
    }

    // Tables
    void addLType(const DRW_LType& ltype) override {}

    void addLayer(const DRW_Layer& layer) override {
        PyLayer pl;
        pl.name = layer.name;
        pl.color = layer.color;
        pl.is_on = layer.plotF;
        pl.lineType = layer.lineType;
        data.layers.push_back(pl);
        data.totalLayers++;
    }

    void addDimStyle(const DRW_Dimstyle& dimstyle) override {}
    void addVport(const DRW_Vport& vport) override {}
    void addTextStyle(const DRW_Textstyle& textstyle) override {}
    void addAppId(const DRW_AppId& appid) override {}

    // Blocks
    void addBlock(const DRW_Block& block) override {
        PyBlock pb;
        pb.name = block.name;
        pb.basePoint = PyCoord(block.basePoint);
        pb.flags = block.flags;
        data.blocks.push_back(pb);
        data.totalBlocks++;

        currentBlockName = block.name;
        inBlock = true;

        // Note: blockMap will be rebuilt after all blocks are added
        // because vector reallocation invalidates pointers
    }

    void setBlock(const int handle) override {}

    void endBlock() override {
        inBlock = false;
        currentBlockName = "";
    }

    // Get current block pointer - search by name
    PyBlock* getCurrentBlock() {
        if (inBlock && !currentBlockName.empty()) {
            // Search in vector (safer than map with pointers)
            for (auto& block : data.blocks) {
                if (block.name == currentBlockName) {
                    return &block;
                }
            }
        }
        return nullptr;
    }

    // Build block map after all blocks are loaded
    void buildBlockMap() {
        blockMap.clear();
        for (auto& block : data.blocks) {
            blockMap[block.name] = &block;
        }
    }

    // Entities
    void addPoint(const DRW_Point& point) override {
        data.totalEntities++;
    }

    void addLine(const DRW_Line& line) override {
        PyLine pl;
        pl.start = PyCoord(line.basePoint);
        pl.end = PyCoord(line.secPoint);
        pl.layer = line.layer;

        if (inBlock) {
            PyBlock* block = getCurrentBlock();
            if (block) {
                block->lines.push_back(pl);
            }
        } else {
            data.lines.push_back(pl);
        }
        data.totalEntities++;
    }

    void addRay(const DRW_Ray& ray) override {
        data.totalEntities++;
    }

    void addXline(const DRW_Xline& xline) override {
        data.totalEntities++;
    }

    void addArc(const DRW_Arc& arc) override {
        PyArc pa;
        pa.center = PyCoord(arc.basePoint);
        pa.radius = arc.radious;
        pa.startAngle = arc.staangle;
        pa.endAngle = arc.endangle;
        pa.layer = arc.layer;

        if (inBlock) {
            PyBlock* block = getCurrentBlock();
            if (block) {
                block->arcs.push_back(pa);
            }
        } else {
            data.arcs.push_back(pa);
        }
        data.totalEntities++;
    }

    void addCircle(const DRW_Circle& circle) override {
        PyCircle pc;
        pc.center = PyCoord(circle.basePoint);
        pc.radius = circle.radious;
        pc.layer = circle.layer;

        if (inBlock) {
            PyBlock* block = getCurrentBlock();
            if (block) {
                block->circles.push_back(pc);
            }
        } else {
            data.circles.push_back(pc);
        }
        data.totalEntities++;
    }

    void addEllipse(const DRW_Ellipse& ellipse) override {
        PyEllipse pe;
        pe.center = PyCoord(ellipse.basePoint);
        pe.majorAxis = PyCoord(ellipse.secPoint);  // Major axis endpoint relative to center
        pe.ratio = ellipse.ratio;
        pe.startAngle = ellipse.staparam;
        pe.endAngle = ellipse.endparam;
        pe.layer = ellipse.layer;

        if (inBlock) {
            PyBlock* block = getCurrentBlock();
            if (block) {
                block->ellipses.push_back(pe);
            }
        } else {
            data.ellipses.push_back(pe);
        }
        data.totalEntities++;
    }

    void addLWPolyline(const DRW_LWPolyline& lwpoly) override {
        PyLWPolyline pp;
        pp.closed = (lwpoly.flags & 1) != 0;
        pp.layer = lwpoly.layer;
        pp.elevation = lwpoly.elevation;

        for (auto* v : lwpoly.vertlist) {
            PyPolylineVertex pv;
            pv.x = v->x;
            pv.y = v->y;
            pv.z = 0;
            pv.bulge = v->bulge;
            pp.vertices.push_back(pv);
        }

        if (inBlock) {
            PyBlock* block = getCurrentBlock();
            if (block) {
                block->lwpolylines.push_back(pp);
            }
        } else {
            data.lwpolylines.push_back(pp);
        }
        data.totalEntities++;
    }

    void addPolyline(const DRW_Polyline& polyline) override {
        data.totalEntities++;
    }

    void addSpline(const DRW_Spline* spline) override {
        data.totalEntities++;
    }

    void addKnot(const DRW_Entity& entity) override {}

    void addInsert(const DRW_Insert& insert) override {
        PyInsert pi;
        pi.blockName = insert.name;
        pi.position = PyCoord(insert.basePoint);
        pi.xScale = insert.xscale;
        pi.yScale = insert.yscale;
        pi.zScale = insert.zscale;
        pi.angle = insert.angle;
        pi.layer = insert.layer;

        // Debug log for inserts (controlled by g_debugEnabled)
        if (g_debugEnabled) {
            static int insertCount = 0;
            insertCount++;
            if (insertCount <= 20 || insert.layer == "WINDOW") {
                std::cerr << "[DEBUG INSERT #" << insertCount << "] layer=" << insert.layer
                          << " block=" << insert.name
                          << " pos=(" << insert.basePoint.x << "," << insert.basePoint.y << ")"
                          << " inBlock=" << (inBlock ? "true" : "false") << std::endl;
            }
        }

        if (inBlock) {
            PyBlock* block = getCurrentBlock();
            if (block) {
                block->inserts.push_back(pi);
            }
        } else {
            data.inserts.push_back(pi);
        }
        data.totalEntities++;
    }

    void addTrace(const DRW_Trace& trace) override {
        data.totalEntities++;
    }

    void add3dFace(const DRW_3Dface& face) override {
        data.totalEntities++;
    }

    void addSolid(const DRW_Solid& solid) override {
        data.totalEntities++;
    }

    void addMText(const DRW_MText& mtext) override {
        PyMText pm;
        pm.content = mtext.text;
        pm.position = PyCoord(mtext.basePoint);
        pm.height = mtext.height;
        pm.angle = mtext.angle;
        pm.layer = mtext.layer;
        pm.style = mtext.style;
        pm.interlin = mtext.interlin;

        if (inBlock) {
            PyBlock* block = getCurrentBlock();
            if (block) {
                block->mtexts.push_back(pm);
            }
        } else {
            data.mtexts.push_back(pm);
        }
        data.totalEntities++;
    }

    void addText(const DRW_Text& text) override {
        PyText pt;
        pt.content = text.text;
        pt.position = PyCoord(text.basePoint);
        pt.height = text.height;
        pt.angle = text.angle;
        pt.layer = text.layer;
        pt.style = text.style;

        if (inBlock) {
            PyBlock* block = getCurrentBlock();
            if (block) {
                block->texts.push_back(pt);
            }
        } else {
            data.texts.push_back(pt);
        }
        data.totalEntities++;
    }

    void addDimAlign(const DRW_DimAligned* dim) override {
        if (!inBlock && dim) {
            PyDimension pd;
            pd.type = dim->type;
            pd.defPoint = PyCoord(dim->getDefPoint());
            pd.textPoint = PyCoord(dim->getTextPoint());
            pd.def1 = PyCoord(dim->getDef1Point());
            pd.def2 = PyCoord(dim->getDef2Point());
            pd.text = dim->getText();
            pd.style = dim->getStyle();
            pd.layer = dim->layer;
            pd.blockName = dim->getName();  // *D block name
            data.dimensions.push_back(pd);
        }
        data.totalEntities++;
    }

    void addDimLinear(const DRW_DimLinear* dim) override {
        if (!inBlock && dim) {
            PyDimension pd;
            pd.type = dim->type;
            pd.defPoint = PyCoord(dim->getDefPoint());
            pd.textPoint = PyCoord(dim->getTextPoint());
            pd.def1 = PyCoord(dim->getDef1Point());
            pd.def2 = PyCoord(dim->getDef2Point());
            pd.angle = dim->getAngle();
            pd.text = dim->getText();
            pd.style = dim->getStyle();
            pd.layer = dim->layer;
            pd.blockName = dim->getName();  // *D block name
            data.dimensions.push_back(pd);
        }
        data.totalEntities++;
    }

    void addDimRadial(const DRW_DimRadial* dim) override {
        if (!inBlock && dim) {
            PyDimension pd;
            pd.type = dim->type;
            pd.defPoint = PyCoord(dim->getCenterPoint());
            pd.textPoint = PyCoord(dim->getTextPoint());
            pd.circlePoint = PyCoord(dim->getDiameterPoint());
            pd.length = dim->getLeaderLength();
            pd.text = dim->getText();
            pd.style = dim->getStyle();
            pd.layer = dim->layer;
            pd.blockName = dim->getName();  // *D block name
            data.dimensions.push_back(pd);
        }
        data.totalEntities++;
    }

    void addDimDiametric(const DRW_DimDiametric* dim) override {
        if (!inBlock && dim) {
            PyDimension pd;
            pd.type = dim->type;
            pd.defPoint = PyCoord(dim->getDiameter2Point());
            pd.textPoint = PyCoord(dim->getTextPoint());
            pd.circlePoint = PyCoord(dim->getDiameter1Point());
            pd.length = dim->getLeaderLength();
            pd.text = dim->getText();
            pd.style = dim->getStyle();
            pd.layer = dim->layer;
            pd.blockName = dim->getName();  // *D block name
            data.dimensions.push_back(pd);
        }
        data.totalEntities++;
    }

    void addDimAngular(const DRW_DimAngular* dim) override {
        if (!inBlock && dim) {
            PyDimension pd;
            pd.type = dim->type;
            pd.defPoint = PyCoord(dim->getSecondLine2());
            pd.textPoint = PyCoord(dim->getTextPoint());
            pd.def1 = PyCoord(dim->getFirstLine1());
            pd.def2 = PyCoord(dim->getFirstLine2());
            pd.circlePoint = PyCoord(dim->getSecondLine1());
            pd.arcPoint = PyCoord(dim->getDimPoint());
            pd.text = dim->getText();
            pd.style = dim->getStyle();
            pd.layer = dim->layer;
            pd.blockName = dim->getName();  // *D block name
            data.dimensions.push_back(pd);
        }
        data.totalEntities++;
    }

    void addDimAngular3P(const DRW_DimAngular3p* dim) override {
        if (!inBlock && dim) {
            PyDimension pd;
            pd.type = dim->type;
            pd.defPoint = PyCoord(dim->getDimPoint());
            pd.textPoint = PyCoord(dim->getTextPoint());
            pd.def1 = PyCoord(dim->getFirstLine());
            pd.def2 = PyCoord(dim->getSecondLine());
            pd.circlePoint = PyCoord(dim->getVertexPoint());
            pd.text = dim->getText();
            pd.style = dim->getStyle();
            pd.layer = dim->layer;
            pd.blockName = dim->getName();  // *D block name
            data.dimensions.push_back(pd);
        }
        data.totalEntities++;
    }

    void addDimOrdinate(const DRW_DimOrdinate* dim) override {
        if (!inBlock && dim) {
            PyDimension pd;
            pd.type = dim->type;
            pd.defPoint = PyCoord(dim->getDefPoint());
            pd.textPoint = PyCoord(dim->getTextPoint());
            pd.text = dim->getText();
            pd.style = dim->getStyle();
            pd.layer = dim->layer;
            pd.blockName = dim->getName();  // *D block name
            data.dimensions.push_back(pd);
        }
        data.totalEntities++;
    }

    void addLeader(const DRW_Leader* leader) override {
        data.totalEntities++;
    }

    void addHatch(const DRW_Hatch* hatch) override {
        if (!inBlock && hatch) {
            PyHatch ph;
            ph.name = hatch->name;
            ph.solid = hatch->solid;
            ph.layer = hatch->layer;
            ph.angle = hatch->angle;
            ph.scale = hatch->scale;

            // Extract boundary loops
            for (const auto* loop : hatch->looplist) {
                if (!loop) continue;

                PyHatchLoop pyLoop;
                pyLoop.type = loop->type;
                pyLoop.closed = true;

                // Check if polyline loop (type & 2)
                if (loop->type & 2) {
                    // Polyline boundary - extract from objlist which contains LWPolyline
                    for (const auto* entity : loop->objlist) {
                        if (!entity) continue;
                        if (entity->eType == DRW::LWPOLYLINE) {
                            const DRW_LWPolyline* pline = static_cast<const DRW_LWPolyline*>(entity);
                            for (const auto* v : pline->vertlist) {
                                PyPolylineVertex pv;
                                pv.x = v->x;
                                pv.y = v->y;
                                pv.z = 0;
                                pv.bulge = v->bulge;
                                pyLoop.vertices.push_back(pv);
                            }
                            pyLoop.closed = (pline->flags & 1) != 0;
                        }
                    }
                } else {
                    // Edge boundary - lines, arcs, ellipses, splines
                    for (const auto* entity : loop->objlist) {
                        if (!entity) continue;

                        PyHatchEdge edge;
                        edge.type = 0;
                        edge.radius = 0;
                        edge.startAngle = 0;
                        edge.endAngle = 0;
                        edge.ccw = true;
                        edge.ratio = 1.0;

                        switch (entity->eType) {
                            case DRW::LINE: {
                                const DRW_Line* line = static_cast<const DRW_Line*>(entity);
                                edge.type = 1;  // Line
                                edge.start = PyCoord(line->basePoint);
                                edge.end = PyCoord(line->secPoint);
                                pyLoop.edges.push_back(edge);
                                break;
                            }
                            case DRW::ARC: {
                                const DRW_Arc* arc = static_cast<const DRW_Arc*>(entity);
                                edge.type = 2;  // Arc
                                edge.center = PyCoord(arc->basePoint);
                                edge.radius = arc->radious;
                                edge.startAngle = arc->staangle;
                                edge.endAngle = arc->endangle;
                                edge.ccw = arc->isccw;
                                pyLoop.edges.push_back(edge);
                                break;
                            }
                            case DRW::ELLIPSE: {
                                const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(entity);
                                edge.type = 3;  // Ellipse
                                edge.center = PyCoord(ellipse->basePoint);
                                edge.majorAxis = PyCoord(ellipse->secPoint);
                                edge.ratio = ellipse->ratio;
                                edge.startAngle = ellipse->staparam;
                                edge.endAngle = ellipse->endparam;
                                edge.ccw = ellipse->isccw;
                                pyLoop.edges.push_back(edge);
                                break;
                            }
                            default:
                                // Skip unsupported edge types (splines, etc.)
                                break;
                        }
                    }
                }

                ph.loops.push_back(pyLoop);
            }

            data.hatches.push_back(ph);
        }
        data.totalEntities++;
    }

    void addViewport(const DRW_Viewport& viewport) override {
        data.totalEntities++;
    }

    void addImage(const DRW_Image* image) override {
        data.totalEntities++;
    }

    void linkImage(const DRW_ImageDef* imagedef) override {}

    void addComment(const char* comment) override {}

    // Write methods (not used for reading)
    void writeHeader(DRW_Header& header) override {}
    void writeBlocks() override {}
    void writeBlockRecords() override {}
    void writeEntities() override {}
    void writeLTypes() override {}
    void writeLayers() override {}
    void writeTextstyles() override {}
    void writeVports() override {}
    void writeDimstyles() override {}
    void writeAppId() override {}

    // Expand all block references
    void expandBlocks(int maxDepth = 10) {
        // Build block map first
        buildBlockMap();

        // First, copy model space entities to expanded lists
        data.expandedLines = data.lines;
        data.expandedCircles = data.circles;
        data.expandedArcs = data.arcs;
        data.expandedPolylines = data.lwpolylines;
        data.expandedTexts = data.texts;
        data.expandedMTexts = data.mtexts;  // Copy model space MTEXT
        data.expandedEllipses = data.ellipses;

        // Then expand each insert
        for (const auto& insert : data.inserts) {
            expandInsert(insert, PyCoord(0, 0, 0), 1.0, 1.0, 0.0, maxDepth);
        }

        // Expand dimension blocks (*D blocks) and update dimension.text
        expandDimensionBlocks();
    }

    // Expand *D blocks associated with DIMENSION entities
    void expandDimensionBlocks() {
        for (size_t i = 0; i < data.dimensions.size(); ++i) {
            PyDimension& dim = data.dimensions[i];
            if (dim.blockName.empty()) continue;

            auto it = blockMap.find(dim.blockName);
            if (it == blockMap.end()) continue;

            PyBlock* block = it->second;
            if (!block) continue;

            // Expand MTEXT from *D block (dimension text)
            for (size_t j = 0; j < block->mtexts.size(); ++j) {
                const PyMText& mtext = block->mtexts[j];

                // Add to expanded mtexts
                PyMText expanded;
                expanded.content = mtext.content;
                expanded.position = mtext.position;
                expanded.height = mtext.height;
                expanded.angle = mtext.angle;
                expanded.layer = dim.layer;
                expanded.style = mtext.style;
                expanded.interlin = mtext.interlin;
                data.expandedMTexts.push_back(expanded);

                // Update dimension.text if empty
                if (dim.text.empty() && !mtext.content.empty()) {
                    std::string text = mtext.content;
                    // Strip formatting codes like \A1;
                    size_t semicolonPos = text.find(';');
                    if (semicolonPos != std::string::npos && semicolonPos < 5) {
                        text = text.substr(semicolonPos + 1);
                    }
                    dim.text = text;
                }
            }
        }
    }

private:
    void expandInsert(const PyInsert& insert, const PyCoord& parentPos,
                      double parentXScale, double parentYScale, double parentAngle,
                      int depth) {
        if (depth <= 0) return;

        // Find the block
        auto it = blockMap.find(insert.blockName);
        if (it == blockMap.end()) return;

        PyBlock* block = it->second;
        if (!block) return;

        // Calculate combined transform
        double xScale = parentXScale * insert.xScale;
        double yScale = parentYScale * insert.yScale;
        double angle = parentAngle + insert.angle;

        // Transform position
        PyCoord pos = insert.position.transform(parentPos, parentXScale, parentYScale, parentAngle);

        // Expand lines
        for (const auto& line : block->lines) {
            PyLine expanded;
            expanded.start = line.start.transform(pos, xScale, yScale, angle);
            expanded.end = line.end.transform(pos, xScale, yScale, angle);
            expanded.layer = line.layer;
            data.expandedLines.push_back(expanded);
        }

        // Expand circles
        for (const auto& circle : block->circles) {
            PyCircle expanded;
            expanded.center = circle.center.transform(pos, xScale, yScale, angle);
            expanded.radius = circle.radius * std::abs(xScale);  // Assume uniform scale for circles
            expanded.layer = circle.layer;
            data.expandedCircles.push_back(expanded);
        }

        // Expand arcs
        for (const auto& arc : block->arcs) {
            PyArc expanded;
            expanded.center = arc.center.transform(pos, xScale, yScale, angle);
            expanded.radius = arc.radius * std::abs(xScale);
            expanded.startAngle = arc.startAngle + angle * 180.0 / M_PI;
            expanded.endAngle = arc.endAngle + angle * 180.0 / M_PI;
            expanded.layer = arc.layer;
            data.expandedArcs.push_back(expanded);
        }

        // Expand polylines
        for (const auto& poly : block->lwpolylines) {
            PyLWPolyline expanded;
            expanded.closed = poly.closed;
            expanded.layer = poly.layer;
            expanded.elevation = poly.elevation;
            for (const auto& v : poly.vertices) {
                PyCoord vc(v.x, v.y, v.z);
                PyCoord tv = vc.transform(pos, xScale, yScale, angle);
                PyPolylineVertex ev;
                ev.x = tv.x;
                ev.y = tv.y;
                ev.z = tv.z;
                ev.bulge = v.bulge;  // Bulge may need adjustment for scale
                expanded.vertices.push_back(ev);
            }
            data.expandedPolylines.push_back(expanded);
        }

        // Expand texts
        for (const auto& text : block->texts) {
            PyText expanded;
            expanded.content = text.content;
            expanded.position = text.position.transform(pos, xScale, yScale, angle);
            expanded.height = text.height * std::abs(yScale);
            expanded.angle = text.angle + angle * 180.0 / M_PI;
            expanded.layer = text.layer;
            expanded.style = text.style;
            data.expandedTexts.push_back(expanded);
        }

        // Expand mtexts (important for dimension text in *D blocks)
        for (const auto& mtext : block->mtexts) {
            PyMText expanded;
            expanded.content = mtext.content;
            expanded.position = mtext.position.transform(pos, xScale, yScale, angle);
            expanded.height = mtext.height * std::abs(yScale);
            expanded.angle = mtext.angle + angle * 180.0 / M_PI;
            expanded.layer = mtext.layer;
            expanded.style = mtext.style;
            expanded.interlin = mtext.interlin;
            data.expandedMTexts.push_back(expanded);
        }

        // Expand ellipses
        for (const auto& ellipse : block->ellipses) {
            PyEllipse expanded;
            expanded.center = ellipse.center.transform(pos, xScale, yScale, angle);
            // Transform major axis - it's relative to center, so just apply scale and rotation
            double majX = ellipse.majorAxis.x * xScale;
            double majY = ellipse.majorAxis.y * yScale;
            double cosA = cos(angle);
            double sinA = sin(angle);
            expanded.majorAxis.x = majX * cosA - majY * sinA;
            expanded.majorAxis.y = majX * sinA + majY * cosA;
            expanded.majorAxis.z = ellipse.majorAxis.z;
            // Ratio may need adjustment if non-uniform scale
            expanded.ratio = ellipse.ratio * (std::abs(yScale) / std::abs(xScale));
            expanded.startAngle = ellipse.startAngle;
            expanded.endAngle = ellipse.endAngle;
            expanded.layer = ellipse.layer;
            data.expandedEllipses.push_back(expanded);
        }

        // Recursively expand nested inserts
        for (const auto& nestedInsert : block->inserts) {
            expandInsert(nestedInsert, pos, xScale, yScale, angle, depth - 1);
        }
    }
};

// Main reader functions
PyDxfData read_dxf(const std::string& filename, bool expand = true) {
    PyDxfCollector collector;
    collector.data.filename = filename;

    dxfRW reader(filename.c_str());

    if (reader.read(&collector, false)) {
        collector.data.valid = true;
        collector.data.version = "DXF";
        if (expand) {
            collector.expandBlocks();
        }
    } else {
        collector.data.valid = false;
        collector.data.error = "Failed to read DXF file";
    }

    return collector.data;
}

PyDxfData read_dwg(const std::string& filename, bool expand = true) {
    PyDxfCollector collector;
    collector.data.filename = filename;

    dwgR reader(filename.c_str());

    if (reader.read(&collector, false)) {
        collector.data.valid = true;

        // Get version string
        DRW::Version ver = reader.getVersion();
        switch (ver) {
            case DRW::AC1006: collector.data.version = "R10"; break;
            case DRW::AC1009: collector.data.version = "R11/R12"; break;
            case DRW::AC1012: collector.data.version = "R13"; break;
            case DRW::AC1014: collector.data.version = "R14"; break;
            case DRW::AC1015: collector.data.version = "R2000"; break;
            case DRW::AC1018: collector.data.version = "R2004"; break;
            case DRW::AC1021: collector.data.version = "R2007"; break;
            case DRW::AC1024: collector.data.version = "R2010"; break;
            case DRW::AC1027: collector.data.version = "R2013"; break;
            case DRW::AC1032: collector.data.version = "R2018"; break;
            default: collector.data.version = "Unknown";
        }

        if (expand) {
            collector.expandBlocks();
        }
    } else {
        collector.data.valid = false;
        DRW::error err = reader.getError();
        switch (err) {
            case DRW::BAD_NONE: collector.data.error = "No error"; break;
            case DRW::BAD_UNKNOWN: collector.data.error = "Unknown error"; break;
            case DRW::BAD_OPEN: collector.data.error = "Cannot open file"; break;
            case DRW::BAD_VERSION: collector.data.error = "Unsupported version"; break;
            case DRW::BAD_READ_FILE_HEADER: collector.data.error = "Bad file header"; break;
            case DRW::BAD_READ_HEADER: collector.data.error = "Bad header"; break;
            case DRW::BAD_READ_HANDLES: collector.data.error = "Bad handles"; break;
            case DRW::BAD_READ_CLASSES: collector.data.error = "Bad classes"; break;
            case DRW::BAD_READ_TABLES: collector.data.error = "Bad tables"; break;
            case DRW::BAD_READ_BLOCKS: collector.data.error = "Bad blocks"; break;
            case DRW::BAD_READ_ENTITIES: collector.data.error = "Bad entities"; break;
            case DRW::BAD_READ_OBJECTS: collector.data.error = "Bad objects"; break;
            default: collector.data.error = "Unknown error code";
        }
    }

    return collector.data;
}

// Python module definition
PYBIND11_MODULE(pydxfrw, m) {
    m.doc() = "Python bindings for libdxfrw - DWG/DXF file reader with block expansion";

    // Coordinate
    py::class_<PyCoord>(m, "Coord")
        .def(py::init<>())
        .def(py::init<double, double, double>())
        .def_readwrite("x", &PyCoord::x)
        .def_readwrite("y", &PyCoord::y)
        .def_readwrite("z", &PyCoord::z)
        .def("__repr__", [](const PyCoord& c) {
            return "Coord(" + std::to_string(c.x) + ", " +
                   std::to_string(c.y) + ", " + std::to_string(c.z) + ")";
        });

    // Layer
    py::class_<PyLayer>(m, "Layer")
        .def(py::init<>())
        .def_readwrite("name", &PyLayer::name)
        .def_readwrite("color", &PyLayer::color)
        .def_readwrite("is_on", &PyLayer::is_on)
        .def_readwrite("line_type", &PyLayer::lineType);

    // Text
    py::class_<PyText>(m, "Text")
        .def(py::init<>())
        .def_readwrite("content", &PyText::content)
        .def_readwrite("position", &PyText::position)
        .def_readwrite("height", &PyText::height)
        .def_readwrite("angle", &PyText::angle)
        .def_readwrite("layer", &PyText::layer)
        .def_readwrite("style", &PyText::style);

    // MText
    py::class_<PyMText>(m, "MText")
        .def(py::init<>())
        .def_readwrite("content", &PyMText::content)
        .def_readwrite("position", &PyMText::position)
        .def_readwrite("height", &PyMText::height)
        .def_readwrite("angle", &PyMText::angle)
        .def_readwrite("layer", &PyMText::layer)
        .def_readwrite("style", &PyMText::style)
        .def_readwrite("interlin", &PyMText::interlin);

    // Line
    py::class_<PyLine>(m, "Line")
        .def(py::init<>())
        .def_readwrite("start", &PyLine::start)
        .def_readwrite("end", &PyLine::end)
        .def_readwrite("layer", &PyLine::layer);

    // Circle
    py::class_<PyCircle>(m, "Circle")
        .def(py::init<>())
        .def_readwrite("center", &PyCircle::center)
        .def_readwrite("radius", &PyCircle::radius)
        .def_readwrite("layer", &PyCircle::layer);

    // Arc
    py::class_<PyArc>(m, "Arc")
        .def(py::init<>())
        .def_readwrite("center", &PyArc::center)
        .def_readwrite("radius", &PyArc::radius)
        .def_readwrite("start_angle", &PyArc::startAngle)
        .def_readwrite("end_angle", &PyArc::endAngle)
        .def_readwrite("layer", &PyArc::layer);

    // Polyline vertex
    py::class_<PyPolylineVertex>(m, "PolylineVertex")
        .def(py::init<>())
        .def_readwrite("x", &PyPolylineVertex::x)
        .def_readwrite("y", &PyPolylineVertex::y)
        .def_readwrite("z", &PyPolylineVertex::z)
        .def_readwrite("bulge", &PyPolylineVertex::bulge);

    // LWPolyline
    py::class_<PyLWPolyline>(m, "LWPolyline")
        .def(py::init<>())
        .def_readwrite("vertices", &PyLWPolyline::vertices)
        .def_readwrite("closed", &PyLWPolyline::closed)
        .def_readwrite("layer", &PyLWPolyline::layer)
        .def_readwrite("elevation", &PyLWPolyline::elevation);

    // Insert (block reference)
    py::class_<PyInsert>(m, "Insert")
        .def(py::init<>())
        .def_readwrite("block_name", &PyInsert::blockName)
        .def_readwrite("position", &PyInsert::position)
        .def_readwrite("x_scale", &PyInsert::xScale)
        .def_readwrite("y_scale", &PyInsert::yScale)
        .def_readwrite("z_scale", &PyInsert::zScale)
        .def_readwrite("angle", &PyInsert::angle)
        .def_readwrite("layer", &PyInsert::layer);

    // Ellipse
    py::class_<PyEllipse>(m, "Ellipse")
        .def(py::init<>())
        .def_readwrite("center", &PyEllipse::center)
        .def_readwrite("major_axis", &PyEllipse::majorAxis)
        .def_readwrite("ratio", &PyEllipse::ratio)
        .def_readwrite("start_angle", &PyEllipse::startAngle)
        .def_readwrite("end_angle", &PyEllipse::endAngle)
        .def_readwrite("layer", &PyEllipse::layer);

    // Block - now includes content
    py::class_<PyBlock>(m, "Block")
        .def(py::init<>())
        .def_readwrite("name", &PyBlock::name)
        .def_readwrite("base_point", &PyBlock::basePoint)
        .def_readwrite("flags", &PyBlock::flags)
        .def_readwrite("lines", &PyBlock::lines)
        .def_readwrite("circles", &PyBlock::circles)
        .def_readwrite("arcs", &PyBlock::arcs)
        .def_readwrite("lwpolylines", &PyBlock::lwpolylines)
        .def_readwrite("texts", &PyBlock::texts)
        .def_readwrite("mtexts", &PyBlock::mtexts)
        .def_readwrite("ellipses", &PyBlock::ellipses)
        .def_readwrite("inserts", &PyBlock::inserts);

    // Dimension with complete geometry
    py::class_<PyDimension>(m, "Dimension")
        .def(py::init<>())
        .def_readwrite("type", &PyDimension::type)
        .def_readwrite("def_point", &PyDimension::defPoint)
        .def_readwrite("text_point", &PyDimension::textPoint)
        .def_readwrite("def1", &PyDimension::def1)
        .def_readwrite("def2", &PyDimension::def2)
        .def_readwrite("circle_point", &PyDimension::circlePoint)
        .def_readwrite("arc_point", &PyDimension::arcPoint)
        .def_readwrite("angle", &PyDimension::angle)
        .def_readwrite("length", &PyDimension::length)
        .def_readwrite("text", &PyDimension::text)
        .def_readwrite("style", &PyDimension::style)
        .def_readwrite("layer", &PyDimension::layer)
        .def_readwrite("block_name", &PyDimension::blockName);

    // Hatch boundary edge
    py::class_<PyHatchEdge>(m, "HatchEdge")
        .def(py::init<>())
        .def_readwrite("type", &PyHatchEdge::type)
        .def_readwrite("start", &PyHatchEdge::start)
        .def_readwrite("end", &PyHatchEdge::end)
        .def_readwrite("center", &PyHatchEdge::center)
        .def_readwrite("radius", &PyHatchEdge::radius)
        .def_readwrite("start_angle", &PyHatchEdge::startAngle)
        .def_readwrite("end_angle", &PyHatchEdge::endAngle)
        .def_readwrite("ccw", &PyHatchEdge::ccw)
        .def_readwrite("major_axis", &PyHatchEdge::majorAxis)
        .def_readwrite("ratio", &PyHatchEdge::ratio);

    // Hatch boundary loop
    py::class_<PyHatchLoop>(m, "HatchLoop")
        .def(py::init<>())
        .def_readwrite("type", &PyHatchLoop::type)
        .def_readwrite("edges", &PyHatchLoop::edges)
        .def_readwrite("vertices", &PyHatchLoop::vertices)
        .def_readwrite("closed", &PyHatchLoop::closed);

    // Hatch with boundary paths
    py::class_<PyHatch>(m, "Hatch")
        .def(py::init<>())
        .def_readwrite("name", &PyHatch::name)
        .def_readwrite("solid", &PyHatch::solid)
        .def_readwrite("layer", &PyHatch::layer)
        .def_readwrite("angle", &PyHatch::angle)
        .def_readwrite("scale", &PyHatch::scale)
        .def_readwrite("loops", &PyHatch::loops);

    // Main data container - now includes expanded entities
    py::class_<PyDxfData>(m, "DxfData")
        .def(py::init<>())
        .def_readwrite("filename", &PyDxfData::filename)
        .def_readwrite("version", &PyDxfData::version)
        .def_readwrite("valid", &PyDxfData::valid)
        .def_readwrite("error", &PyDxfData::error)
        .def_readwrite("layers", &PyDxfData::layers)
        .def_readwrite("texts", &PyDxfData::texts)
        .def_readwrite("mtexts", &PyDxfData::mtexts)
        .def_readwrite("lines", &PyDxfData::lines)
        .def_readwrite("circles", &PyDxfData::circles)
        .def_readwrite("arcs", &PyDxfData::arcs)
        .def_readwrite("lwpolylines", &PyDxfData::lwpolylines)
        .def_readwrite("ellipses", &PyDxfData::ellipses)
        .def_readwrite("inserts", &PyDxfData::inserts)
        .def_readwrite("blocks", &PyDxfData::blocks)
        .def_readwrite("dimensions", &PyDxfData::dimensions)
        .def_readwrite("hatches", &PyDxfData::hatches)
        // Expanded entities
        .def_readwrite("expanded_lines", &PyDxfData::expandedLines)
        .def_readwrite("expanded_circles", &PyDxfData::expandedCircles)
        .def_readwrite("expanded_arcs", &PyDxfData::expandedArcs)
        .def_readwrite("expanded_polylines", &PyDxfData::expandedPolylines)
        .def_readwrite("expanded_texts", &PyDxfData::expandedTexts)
        .def_readwrite("expanded_mtexts", &PyDxfData::expandedMTexts)
        .def_readwrite("expanded_ellipses", &PyDxfData::expandedEllipses)
        .def_readwrite("total_entities", &PyDxfData::totalEntities)
        .def_readwrite("total_layers", &PyDxfData::totalLayers)
        .def_readwrite("total_blocks", &PyDxfData::totalBlocks);

    // Functions
    m.def("read_dxf", [](const std::string& filename) {
        return read_dxf(filename, true);
    }, "Read a DXF file with block expansion",
          py::arg("filename"));

    m.def("read_dwg", [](const std::string& filename) {
        return read_dwg(filename, true);
    }, "Read a DWG file with block expansion",
          py::arg("filename"));

    m.def("read", [](const std::string& filename) -> PyDxfData {
        std::string ext = filename.substr(filename.find_last_of(".") + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == "dwg") {
            return read_dwg(filename, true);
        } else {
            return read_dxf(filename, true);
        }
    }, "Read a DWG or DXF file with block expansion",
       py::arg("filename"));

    // Encoding control functions
    m.def("set_encoding", [](const std::string& encoding) {
        DRW_TextCodec::setFallbackEncoding(encoding);
    }, "Set the fallback encoding for text decoding",
       py::arg("encoding"));

    m.def("get_encoding", []() -> std::string {
        return DRW_TextCodec::getFallbackEncoding();
    }, "Get the current fallback encoding");

    // Read with specific encoding
    m.def("read_dwg_with_encoding", [](const std::string& filename, const std::string& encoding) -> PyDxfData {
        std::string oldEncoding = DRW_TextCodec::getFallbackEncoding();
        DRW_TextCodec::setFallbackEncoding(encoding);
        PyDxfData result = read_dwg(filename, true);
        DRW_TextCodec::setFallbackEncoding(oldEncoding);
        return result;
    }, "Read a DWG file with specific encoding and block expansion",
       py::arg("filename"), py::arg("encoding"));

    m.def("read_with_encoding", [](const std::string& filename, const std::string& encoding) -> PyDxfData {
        std::string ext = filename.substr(filename.find_last_of(".") + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        std::string oldEncoding = DRW_TextCodec::getFallbackEncoding();
        DRW_TextCodec::setFallbackEncoding(encoding);

        PyDxfData result;
        if (ext == "dwg") {
            result = read_dwg(filename, true);
        } else {
            result = read_dxf(filename, true);
        }

        DRW_TextCodec::setFallbackEncoding(oldEncoding);
        return result;
    }, "Read a DWG or DXF file with specific encoding and block expansion",
       py::arg("filename"), py::arg("encoding"));

    // Debug control functions
    m.def("set_debug", &setDebugEnabled, "Enable or disable debug output (default: enabled)",
          py::arg("enabled"));

    m.def("get_debug", &isDebugEnabled, "Get current debug output state");
}
