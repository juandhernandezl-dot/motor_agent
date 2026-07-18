#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
controller_registry.py  -  DESCUBRIMIENTO DINAMICO DE CONTROLADORES
===================================================================

Para que la GUI no tenga una lista fija de controladores. Cualquier .py de la
carpeta que declare un diccionario llamado CONTROLLER a nivel de modulo aparece
solo en la GUI, con su formulario de parametros construido al vuelo.

QUE HAY QUE ANADIR A TU CONTROLADOR (y nada mas)
------------------------------------------------

    CONTROLLER = {
        "clave": "mi_control",           # nombre unico; el que usa en el bus
        "nombre": "Mi controlador",      # como aparece en la GUI
        "familia": "clasico",            # agrupacion libre: RL / clasico / ...
        "kind": "none",                  # "q"|"policy"|"none" -> si publica tabla
        "descripcion": "Una o dos lineas de que hace y cuando usarlo.",
        "params": [
            {"tipo": "float", "etiqueta": "Consigna (rpm)", "flag": "--sp",
             "default": 1500, "ayuda": "0 = usar la de la GUI"},
            {"tipo": "int",   "etiqueta": "...", "flag": "--n", "default": 10},
            {"tipo": "bool",  "etiqueta": "...", "flag": "--algo", "default": False},
            {"tipo": "str",   "etiqueta": "...", "flag": "--x", "default": ""},
            {"tipo": "choice","etiqueta": "Modo", "default": "",
             "opciones": [["Preguntar", ""], ["Controlar", "--controlar"]]},
        ],
    }

Reglas:
  * 'flag' sin valor -> tipo bool (se anade la bandera si esta marcada).
  * tipo 'choice' -> cada opcion es [etiqueta, argumentos]; los argumentos se
    anaden tal cual (pueden ser "" para no anadir nada, o "--x 5").
  * un parametro con valor vacio y default None NO se pasa: el script decide.

IMPORTANTE: el manifiesto se lee con 'ast' SIN IMPORTAR el modulo. Es decir,
listar controladores nunca ejecuta su codigo: la GUI no puede arrancar un motor
por el mero hecho de refrescar la lista.
"""

import ast
import os
import shlex
import sys
from dataclasses import dataclass, field

MANIFIESTO = "CONTROLLER"


@dataclass
class Controller:
    ruta: str
    clave: str
    nombre: str
    familia: str = "otros"
    kind: str = "none"
    descripcion: str = ""
    params: list = field(default_factory=list)

    @property
    def fichero(self):
        return os.path.basename(self.ruta)


def _leer_manifiesto(ruta):
    """Extrae el dict CONTROLLER de un .py sin ejecutarlo (solo analisis)."""
    try:
        with open(ruta, "r", encoding="utf-8") as f:
            arbol = ast.parse(f.read(), filename=ruta)
    except (OSError, SyntaxError, ValueError):
        return None
    for nodo in arbol.body:
        if not isinstance(nodo, ast.Assign):
            continue
        for destino in nodo.targets:
            if isinstance(destino, ast.Name) and destino.id == MANIFIESTO:
                try:
                    d = ast.literal_eval(nodo.value)
                except (ValueError, SyntaxError):
                    return None
                return d if isinstance(d, dict) else None
    return None


def descubrir(directorios=None, verbose=False):
    """Devuelve los Controller encontrados, ordenados por familia y nombre."""
    if directorios is None:
        directorios = [os.path.dirname(os.path.abspath(__file__))]
    vistos, salida = set(), []
    for d in directorios:
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if not f.endswith(".py") or f.startswith("_"):
                continue
            ruta = os.path.join(d, f)
            man = _leer_manifiesto(ruta)
            if not man or "clave" not in man:
                continue
            if man["clave"] in vistos:
                if verbose:
                    print(f"  [registro] clave duplicada '{man['clave']}' en {f}")
                continue
            vistos.add(man["clave"])
            salida.append(Controller(
                ruta=ruta, clave=man["clave"],
                nombre=man.get("nombre", man["clave"]),
                familia=man.get("familia", "otros"),
                kind=man.get("kind", "none"),
                descripcion=man.get("descripcion", ""),
                params=man.get("params", []) or []))
    salida.sort(key=lambda c: (c.familia, c.nombre))
    return salida


def construir_cmd(ctl: Controller, valores: dict, python=None):
    """
    valores: {indice_del_param: valor_ya_leido_del_formulario}
    Devuelve la lista de argumentos lista para subprocess.Popen.
    """
    cmd = [python or sys.executable, "-u", ctl.ruta]
    for i, p in enumerate(ctl.params):
        v = valores.get(i)
        tipo = p.get("tipo", "str")
        flag = p.get("flag")
        if tipo == "bool":
            if v:
                cmd.append(flag)
        elif tipo == "choice":
            if v:
                cmd.extend(shlex.split(str(v)))
        else:
            if v is None or str(v).strip() == "":
                continue
            if flag:
                cmd.extend([flag, str(v)])
            else:
                cmd.append(str(v))
    return cmd


def main():
    print("Controladores detectados:\n")
    for c in descubrir(verbose=True):
        print(f"  [{c.familia}] {c.nombre}   ({c.fichero}, clave='{c.clave}')")
        if c.descripcion:
            print(f"      {c.descripcion.strip().splitlines()[0]}")
        for p in c.params:
            print(f"      - {p.get('etiqueta','?')} ({p.get('tipo','str')})"
                  f"{'  ' + p['flag'] if p.get('flag') else ''}")
        print()


if __name__ == "__main__":
    main()
