interface Movement {
  x: number;
  y: number;
  z: number;
  th: number;
}

export const enum MovementDirection {
  FORWARD  = 'FORWARD',
  BACKWARD = 'BACKWARD',
  LEFT     = 'LEFT',
  RIGHT    = 'RIGHT',
}

export const MOVEMENTS: Record<MovementDirection, Movement> = {
  FORWARD  : {
    x  : 1,
    y  : 0,
    z  : 0,
    th : 0,
  },
  BACKWARD : {
    x  : -1,
    y  : 0,
    z  : 0,
    th : 0,
  },
  LEFT     : {
    x  : 0,
    y  : 0,
    z  : 0,
    th : 1,

  },
  RIGHT    : {
    x  : 0,
    y  : 0,
    z  : 0,
    th : -1,

  },

};
