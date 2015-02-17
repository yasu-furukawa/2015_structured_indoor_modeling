uniform sampler2D tex0;
uniform sampler2D tex1;
uniform float weight;
uniform int divide_by_alpha;

void main() {
  vec4 rgba0 = texture2D(tex0, gl_TexCoord[0].st);
  vec4 rgba1 = texture2D(tex1, gl_TexCoord[0].st);

  if (divide_by_alpha == 0) {
    gl_FragColor = rgba0 * weight + rgba1 * (1.0 - weight);
  } else if (divide_by_alpha == 1) {
    if (rgba0.r == 0.0 && rgba0.g == 0.0 && rgba0.b == 0.0) {
      gl_FragColor = rgba1;
    } else if (rgba1.r == 0.0 && rgba1.g == 0.0 && rgba1.b == 0.0) {
      gl_FragColor = rgba0;
    } else {
      gl_FragColor = rgba0 * weight + rgba1 * (1.0 - weight);
    }
  } else if (divide_by_alpha == 2) {
    if (rgba0.r == 0.0 && rgba0.g == 0.0 && rgba0.b == 0.0) {
      gl_FragColor = rgba1;
    } else {
      gl_FragColor = rgba0 * weight + rgba1 * (1.0 - weight);
    }
  } else {
    if (rgba1.r == 0.0 && rgba1.g == 0.0 && rgba1.b == 0.0) {
      gl_FragColor = rgba0;
    } else {
      gl_FragColor = rgba0 * weight + rgba1 * (1.0 - weight);
    }
  }





  

  //gl_FragColor = texture2D(tex0, gl_TexCoord[0].st);
  //  gl_FragColor = texture2D(tex0, vec2(0.5, 0.5));
  /*
  gl_FragColor[0] = weight;
  gl_FragColor[1] = weight;
  gl_FragColor[2] = weight;
  */
  //  gl_FragColor[3] = 1.0;



  /*
  float suma = rgba0.a + rgba1.a;
  if (rgba0.a < 0.5) { 
    gl_FragColor = vec4(1, 0, 0, 1);
  }      
  else if (rgba0.a >= 0.5) {
    gl_FragColor = vec4(0, 1, 0, 1);
  }
  else {
    gl_FragColor[0] = (rgba0.r * rgba0.a + rgba1.r * rgba1.a) / suma;
    gl_FragColor[1] = (rgba0.g * rgba0.a + rgba1.g * rgba1.a) / suma;
    gl_FragColor[2] = (rgba0.b * rgba0.a + rgba1.b * rgba1.a) / suma;
    gl_FragColor[3] = 1.0;
  }
  */
  
  /*
  vec2 coord = gl_TexCoord[0];
  vec4 lhs = texture2D(sampler0, coord);
  vec4 rhs = texture2D(sampler1, coord);

  if (lhs[0] == 0 && lhs[1] == 0 && lhs[2] == 0) {
    gl_FragColor = rhs;
  } else if (rhs[0] == 0 && rhs[1] == 0 && rhs[2] == 0) {
    gl_FragColor = lhs;
  } else {
    gl_FragColor = weight * lhs + (1.0 - weight) * rhs;
    }*/
  //gl_FragColor = weight * texture2D(sampler0, gl_TexCoord[0].xy) + (1.0 - weight) * texture2D(sampler1, gl_TexCoord[0].xy);
  //  gl_FragColor = texture2D(sampler0, gl_TexCoord[0].st);

  //gl_FragColor.r = gl_TexCoord[0].s;
}
