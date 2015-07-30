#include "BoundaryGenerator.h"
#include "Utils.h"

Vector2d convex1[] = {Vector2d(300,100), Vector2d(100,300), Vector2d(300,500), Vector2d(500,300)};

Vector2d convex2[] = {Vector2d(100,200), Vector2d(100,400), Vector2d(500,400), Vector2d(500,200)};

Vector2d convex3[] = {Vector2d(100,200), Vector2d(100,400), Vector2d(300,500), Vector2d(500,400), 
					  Vector2d(500,200), Vector2d(300,100)};

Vector2d convex4[] = {Vector2d(100,400), Vector2d(200,500), Vector2d(450,500), Vector2d(500,250), 
					  Vector2d(350,100)};

Vector2d concave1[] = {Vector2d(280,100), Vector2d(130,400), Vector2d(280,300), Vector2d(430,400)};

Vector2d concave2[] = {Vector2d(100,250), Vector2d(250,300), Vector2d(300,450), Vector2d(350,300), 
					   Vector2d(500,250), Vector2d(350,200), Vector2d(300,50), Vector2d(250,200)};

Vector2d concave3[] = {Vector2d(200,100), Vector2d(200,400), Vector2d(400,400), Vector2d(400,350),
                       Vector2d(250,350), Vector2d(250,275), Vector2d(400,275), Vector2d(400,200),
                       Vector2d(250,200), Vector2d(250,150), Vector2d(400,150), Vector2d(400,100)};

Vector2d concave4[] = {Vector2d(160,440), Vector2d(420,440), Vector2d(340,320), Vector2d(460,400),
                       Vector2d(460,140), Vector2d(340,220), Vector2d(420,100), Vector2d(160,100),
                       Vector2d(240,220), Vector2d(120,140), Vector2d(120,400), Vector2d(240,320)};

Vector2d concave5[] = {Vector2d(200,200), Vector2d(200,400), Vector2d(400,325), Vector2d(350,300),
                       Vector2d(400,275)};

Vector2d concave6[] = {Vector2d(269,553), Vector2d(316,552), Vector2d(321,366), Vector2d(356,313),
                       Vector2d(424,332), Vector2d(395,280), Vector2d(480,278), Vector2d(453,250),
                       Vector2d(485,204), Vector2d(453,202), Vector2d(485,157), Vector2d(370,148),
                       Vector2d(392,103), Vector2d(308,125), Vector2d(289,186), Vector2d(228,134),
                       Vector2d(143,109), Vector2d(164,155), Vector2d(104,159), Vector2d(167,185),
                       Vector2d(63,234),  Vector2d(172,230), Vector2d(95,289),  Vector2d(157,284),
                       Vector2d(119,349), Vector2d(223,328), Vector2d(185,386), Vector2d(260,360),
                       Vector2d(275,468)};


std::vector<BoundaryElement> BoundaryGenerator::getBoundaryElements(unsigned char shape)
{
	// clear the vector
	boundaryElements.clear();

	// generate the predefined shape
	switch (shape) {
		case 'q':
			generateShape(convex1, sizeof(convex1) / sizeof(convex1[0]));
			break;
		case 'w':
			generateShape(convex2, sizeof(convex2) / sizeof(convex2[0]));	
			break;
		case 'e':
			generateShape(convex3, sizeof(convex3) / sizeof(convex3[0]));	
			break;
		case 'r':
			generateShape(convex4, sizeof(convex4) / sizeof(convex4[0]));	
			break;
		case 't':
			generateShape(concave1, sizeof(concave1) / sizeof(concave1[0]));	
			break;
		case 'y':
			generateShape(concave2, sizeof(concave2) / sizeof(concave2[0]));	
			break;
		case 'u':
			generateShape(concave3, sizeof(concave3) / sizeof(concave3[0]));
			break;
        case 'i':
            generateShape(concave4, sizeof(concave4) / sizeof(concave4[0]));
            break;
        case 'o':
            generateShape(concave5, sizeof(concave5) / sizeof(concave5[0]));
            break;
        case 'p':
            generateShape(concave6, sizeof(concave6) / sizeof(concave6[0]));
            break;
	}

	return boundaryElements;
}

void BoundaryGenerator::generateShape(const Vector2d *shape, int vertexCount)
{
	// create edges from the vertex arrays
	std::vector<Edge> edges;
	for (int i = 0; i < vertexCount; i++) {
		int next = (i+1) % vertexCount;		
		edges.push_back(Edge(shape[i], shape[next]));
	}

	// create boundary elements with halflines and add them to the boundaryElements vector 
	int index = 0;
	for (int i = 0; i < (int)edges.size(); i++) {
		int prev = i-1;
		if (prev < 0) prev = vertexCount-1;

		int next = (i+1) % vertexCount;		

		BoundaryElement be1, be2, be3;
		if (prev != vertexCount-1 && Utils::inHalfPlane(edges[prev].tangent(), edges[i].tangent())) {
			be1 = BoundaryElement(edges[i].vertex1, edges[prev].normal(), edges[i].normal(), index);
		}

		if (Utils::inHalfPlane(edges[i].tangent(), edges[next].tangent())) {
			be3 = BoundaryElement(edges[i].vertex2, edges[i].normal(), edges[next].normal(), index+1);
		}

		if (be1.index != -1 && be3.index != -1) {
			be2 = BoundaryElement(edges[i], be1.halfLine2, be3.halfLine1, index);

			boundaryElements.push_back(be2);
			boundaryElements.push_back(be3);
			index++;

		} else if (be1.index != -1) {
			Vector2d halfLine2 = (-edges[i].tangent() + edges[next].tangent())/2;
			halfLine2.normalize();

			be2 = BoundaryElement(edges[i], be1.halfLine2, halfLine2, index);

			boundaryElements.push_back(be2);

		} else if (be3.index != -1) {
			Vector2d halfLine1 = (-edges[prev].tangent() + edges[i].tangent())/2;
			halfLine1.normalize();

			be2 = BoundaryElement(edges[i], halfLine1, be3.halfLine1, index);

			boundaryElements.push_back(be2);
			boundaryElements.push_back(be3);
			index++;

		} else {
			Vector2d halfLine1 = (-edges[prev].tangent() + edges[i].tangent())/2;
			halfLine1.normalize();

			Vector2d halfLine2 = (-edges[i].tangent() + edges[next].tangent())/2;
			halfLine2.normalize();

			be2 = BoundaryElement(edges[i], halfLine1, halfLine2, index);

			boundaryElements.push_back(be2);
		}

		index++;
	}
}
