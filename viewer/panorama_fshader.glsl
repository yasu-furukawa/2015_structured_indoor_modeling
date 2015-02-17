uniform sampler2D tex0;
uniform float phi_range;
uniform float alpha;

varying vec3 local;


void main() {
     float PI = 3.1415926535897932384626433832795;

     float theta = - atan(local.y, local.x);
     if (theta < 0.0) {
        theta = theta + 2.0 * PI;
     }
     vec2 uv;

     uv.x = theta / (2.0 * PI);
     
     float depth = sqrt(local.x * local.x + local.y * local.y);
     float phi = atan(local.z, depth);
     float pixel_offset_from_center = phi / phi_range;
     uv.y = max(0.0, min(1.0, 0.5 + pixel_offset_from_center));

     gl_FragColor = alpha * texture2D(tex0, uv);
}
